/*
 * S88.cpp
 *
 *  Created on: Mar 11, 2021
 *      Author: sdedic
 */

#include <Arduino.h>
#include "Defs.h"
#include "S88.h"
#include "Common.h"
#include "Utils.h"
#include "Terminal.h"

byte s88Sensorstates[s88MaxSize_bytes] = { 0 };
boolean s88BusChanged = false;

// Shamelessly copied from https://sites.google.com/site/sidloweb/elektrika/s88-ir-detektor
// Copyright (c) Sidlo
byte data = 0 ;                   	// data byte
int bitCounter = 0 ;              	// bit counter
short byteIndex = 0;

/**
 * Sensor table. The table is accessed in an interrupt, so no moves are permitted to
 * avoid sync issues.
 */
Sensor sensors[maxSensorCount];

/**
 * Millis counter at the last LOAD interrupt.
 */
long lastS88Millis = 0;

/**
 * The actual sensor count, must be <= maxSensorCount.
 */
int sensorCount = 0;

sensorChangeFunc sensorCallback = NULL;

Sensor::Sensor(const SensorData& d) :
	reportState(false), s88State(false), triggerChange(false), changing(false), changeProcessing(false), overriden(false) {
	sensorId = d.sensorId;
	sensorDownDebounce = d.sensorDownDebounce;
	sensorUpDebounce = d.sensorUpDebounce;
	triggerSensor = d.triggerSensor;
}

SensorData::SensorData(const Sensor& s) {
	sensorId = s.sensorId;
	sensorDownDebounce = s.sensorDownDebounce;
	sensorUpDebounce = s.sensorUpDebounce;
	triggerSensor = s.triggerSensor;
}

int Sensor::upDebounceTime() const {
	if (sensorUpDebounce == 0) {
		return triggerSensor ? defaultTiming.triggerUpDebounce : defaultTiming.trackUpDebounce;
	} else {
		return sensorUpDebounce;
	}
}

int Sensor::downDebounceTime() const {
	if (sensorDownDebounce == 0) {
		return triggerSensor ? defaultTiming.triggerDownDebounce : defaultTiming.trackDownDebounce;
	} else {
		return sensorUpDebounce;
	}
}

void Sensor::dumpTimeouts() const {
	if ((sensorUpDebounce == 0) && (sensorDownDebounce == 0)) {
		return;
	}
	Serial.print(F("STM:")); Serial.print(sensorId);
	if (sensorDownDebounce > 0) {
		Serial.print(F(":D=")); Serial.print(sensorDownDebounce);
	}
	if (sensorDownDebounce > 0) {
		Serial.print(F(":U=")); Serial.print(sensorUpDebounce);
	}
}

void Sensor::printAll(boolean includeNone) {
	for (int i = 0; i < maxSensorCount; i++) {
		const Sensor& s = sensors[i];
		if (includeNone || s.isDefined()) {
			s.print();
		}
	}
}

void Sensor::print() const {
	Serial.print(F("Sensor #"));
	Serial.print((this - sensors));
	Serial.print("("); Serial.print(sensorId);
	Serial.print(F("): state(")); Serial.print(reportState);
	Serial.print(F("), s88(")); Serial.print(s88State); Serial.print(")");
	if (changing) {
		long ms = millis() & 0xffff;
		if (ms < stableFrom) {
			ms += 0x1000;
		}
		Serial.print(F(" - millis: ")); Serial.print(ms - stableFrom);
	}
	Serial.println();
}

boolean defineSensor(int id, boolean trigger) {
	if (sensorCount >= maxSensorCount) {
		return false;
	}
	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId == id) {
			s.triggerSensor = trigger;
			return true;
		} else if (s.sensorId <= 0) {
			s = Sensor(id);
			s.triggerSensor = trigger;
			sensorCount++;
			return true;
		}
	}
	return false;
}

boolean defineSensor(int id) {
	return defineSensor(id, false);
}

int forSensors(sensorIteratorFunc fn) {
	int cnt = 0;
	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId > 0) {
			cnt += fn(s.sensorId, s.triggerSensor);
		}
	}
	return cnt;
}

boolean freeSensor(int id) {
	if (sensorCount == 0) {
		return false;
	}
	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId == id) {
			s.clear();
			sensorCount--;
			return true;
		}
	}
	return false;
}

boolean s88Changed(int sensor) {
	for (int i = 0; i < sensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId == sensor) {
			return s.triggerChange || s.changeProcessing;
		}
	}
	return false;
}

void s88InLoop() {
	for (int i = 0; i < sensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.triggerChange) {
			s.changeProcessing = true;
			s.triggerChange = false;
		}
	}
	unsigned long processed = 0x00;
	unsigned long mask = 0x01;

	for (int i = 0; i < sensorCount; i++, mask = mask << 1) {
		Sensor& s = sensors[i];
		if (s.changeProcessing) {
			s.triggerChange = false;
			if (s.suspended) {
				continue;
			}
			// retain "changing" for this process cycle.
			processed |= mask;
			if (debugS88) {
				Serial.print(F("Sensor ")); Serial.print(s.sensorId); Serial.print(" trigger:"); Serial.print(s.triggerSensor);
				Serial.print(F(" changed to: ")); Serial.print(s.reportState);
				Serial.print(F(" Reported after "));
				unsigned long l = lastS88Millis & 0xffff;
				if (l < s.stableFrom) {
					l += 0x10000;
				}
				Serial.println(l - s.stableFrom);
			}
			if (sensorCallback) {
				sensorCallback(s.sensorId, s.s88State);
			}
		}
	}
	mask = 0x01;
	for (int i = 0; i < sensorCount; i++, mask = mask << 1) {
		if ((processed & mask) > 0) {
			Sensor& s = sensors[i];
			s.changeProcessing = false;
		}
	}
}

// ==================== Routines run in the interrupt ======================
long millisQuantum = 50;

void storeS88Bit(int sensorId, int state, boolean skipOverride) {
	byte stateIdx = (sensorId - 1) / 8;
	byte stateMask = 1 << ((sensorId -1) % 8);

	boolean cur = (s88Sensorstates[stateIdx] & stateMask) > 0;
	if (cur != state) {
		s88BusChanged = true;
	}
	s88Sensorstates[stateIdx] = state ?
		s88Sensorstates[stateIdx] | stateMask :
		s88Sensorstates[stateIdx] & ~stateMask;

	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];

		if (s.sensorId != sensorId) {
			continue;
		}
		if (s.overriden && skipOverride) {
			s.s88State = state;
			continue;
		}
		long l;

		if (s.changing) {
			l = lastS88Millis & 0xffff;
			if (l < s.stableFrom) {
				l += 0x10000;
			}
			l = l - s.stableFrom;
		} else {
			l = 0;
		}
		if (state == s.reportState) {
			if (debugS88Debounce) {
				if (s.changing) {
					Serial.print(F("Sensor ")); Serial.print(sensorId); Serial.print(F(" reset after ")); Serial.println(l);
				}
			}
			s.s88State = state;
			s.changing = false;
			return;
		}
		int deb = state ? s.upDebounceTime() : s.downDebounceTime();
		if (state == s.s88State) {
			if (!s.changing) {
				return;
			}
			if (l >= deb) {
				if (debugS88Debounce) {
					Serial.print(F("Sensor ")); Serial.print(sensorId); Serial.print(F(" TRIGGER to ")); Serial.print(state);
					Serial.print(F(" after ")); Serial.println(l);
				}
				s.changing = false;
				s.triggerChange = true;
				s.reportState = state;
			} else if (debugS88Debounce && s.changing) {
				Serial.print(F("Sensor ")); Serial.print(sensorId); Serial.print(F( "steady: ")); Serial.println(l);
			}
		} else if (millisQuantum > deb) {
			if (debugS88Debounce) {
				Serial.print(F("Sensor ")); Serial.print(sensorId); Serial.print(F(" TRIGGER to ")); Serial.println(state);
			}
			s.s88State = state;
			s.changing = false;
			s.triggerChange = true;
			s.reportState = state;
			s.stableFrom = lastS88Millis & 0xffff;
		} else {
			s.s88State = state;
			s.changing = true;
			s.stableFrom = lastS88Millis & 0xffff;
			if (debugS88Debounce) {
				Serial.print(F("Sensor ")); Serial.print(sensorId); Serial.print(F(" changing to "));
				Serial.print(state); Serial.print(F(" at millis ")); Serial.println(s.stableFrom);
			}
		}
	}
}

long prevS88 = 0;
long cummulativeS88 = 0;
long s88IntCount = 0;


/***************************************************************************
 * Interrupt 0 LOAD.
 */
void s88LoadInt() {
  bitCounter = 0 ;
  lastS88Millis = millis();
  if (prevS88 > 0) {
	  s88IntCount++;
	  if ((s88IntCount % 100) == 0) {
		  millisQuantum = cummulativeS88 / s88IntCount;
		  cummulativeS88 = 0;
		  s88IntCount = 1;
	  }
	  cummulativeS88 += (lastS88Millis - prevS88);
  }
  prevS88 = lastS88Millis;
  byteIndex = 0;
}

/***************************************************************************
 * Interrupt 1 CLOCK.
 */
void s88ClockInt() {
  int xferBit = (bitCounter++) % 8;

  // read input
  int x = digitalRead(DATA_IN);

  // send out the same bit
  digitalWrite(DATA_OUT, x) ;

  storeS88Bit(bitCounter, x > 0, true);

  if (debugS88Low) {
    if ((bitCounter  % 8) == 0 && bitCounter > 7 && data != 0) {
      Serial.print(F("S88 read byte: ")); Serial.print((bitCounter - 1)/ 8 + 65); Serial.print(F(" = "));
      Serial.println(data);
    }
  }
}

void setS88Sensor(int sensor, int state) {
}

int tryReadS88(int sensor) {
	if (sensor < 0 || sensor >= (s88MaxSize * 8)) {
		return -1;
	}
	for (int i = 0; i < sensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId == sensor) {
			if (s.suspended) {
				return s.suspendedState ? 1 : 0;
			} else {
				return s.reportState ? 1 : 0;
			}
		}
	}
	return -1;
}

boolean readS88(int sensor) {
	int r = tryReadS88(sensor);
	return r > 0;
}

void suspendS88(int sensorId) {
	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId == sensorId) {
			s.suspendedState = s.reportState;
			s.suspended = true;
			break;
		}
	}
}

void resumeS88(int sensorId) {
	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId == sensorId) {
			s.suspended = false;
			break;
		}
	}
}


void overrideS88(int sensorId, boolean override, boolean state) {
	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];
		if (s.sensorId == sensorId) {
			boolean change;
			if (override) {
				change = tryReadS88(sensorId) != (state ? 1 : 0);
				s.overriden = true;
				s.reportState = state;
				if (debugS88) {
					Serial.print(F("Sensor ")); Serial.print(sensorId);
					Serial.print(F(" set to ")); Serial.println(state);
				}
			} else {
				change = s.reportState != s.s88State;
				s.reportState = s.s88State;
				if (debugS88) {
					Serial.print(F("Sensor ")); Serial.print(sensorId);
					Serial.println(F(" released"));
				}
			}
			if (change) {
				if (debugS88) {
					Serial.println(F("Trigger."));
				}
				s.triggerChange = true;
			}
			return;
		}
	}
	Serial.print(F("No sensor: ")); Serial.println(sensorId);
}

void resetAllSensors() {
	Serial.println(F("Resetting sensor defs"));
	for (int i = 0; i < maxSensorCount; i++) {
		sensors[i] = Sensor();
	}
	sensorCount = 0;
}


boolean loadEEPROMSensors() {
	int addr = eepromSensors;
	int checksum = 0;
	boolean allzero;
	sensorCount = 0;
	for (int i = 0; i < maxSensorCount; i++) {
		Sensor& s = sensors[i];
		s = Sensor();
		s.sensorId = eepromReadByte(addr, checksum, allzero);
		s.triggerSensor = eepromReadByte(addr, checksum, allzero);
		s.sensorUpDebounce = eepromReadInt(addr, checksum, allzero);
		s.sensorDownDebounce = eepromReadInt(addr, checksum, allzero);
		if (s.sensorId > 0) {
			sensorCount++;
		}
	}
	int tmp = 0;
	int ch = eepromReadInt(addr, tmp, allzero);
	if (ch != checksum) {
		Serial.println(F("Sensor corrupted."));
		resetAllSensors();
	}
	return true;
}

void saveEEPROMSensors() {
	Serial.println(F("Saving sensors"));
	int addr = eepromSensors;
	int checksum = 0;
	for (int i = 0; i < maxSensorCount; i++) {
		const Sensor& s = sensors[i];

		eepromWriteByte(addr++, s.sensorId, checksum);
		eepromWriteByte(addr++, s.triggerSensor, checksum);
		addr = eepromWriteInt(addr, s.sensorUpDebounce, checksum);
		addr = eepromWriteInt(addr, s.sensorDownDebounce, checksum);
	}
	int tmp = 0;
	eepromWriteInt(addr, checksum, tmp);
}

void s88Status() {
	Serial.println(F("Sensor status:"));
	for (int i = 0; i < maxSensorCount; i++) {
		const Sensor& s = sensors[i];
		if (s.sensorId == 0) {
			continue;
		}
		Serial.print(s.sensorId);
		Serial.print(F(":\tr=")); Serial.print(s.reportState);
		Serial.print(F(":\ts=")); Serial.print(s.s88State);
		Serial.print(F(":\tt=")); Serial.print(s.triggerChange);
		Serial.print(F(":\to=")); Serial.print(s.overriden);
		Serial.print(F(":\tfrom=")); Serial.print(s.stableFrom);
		Serial.println();
	}
}

void cmdReleaseSensors() {
	int n;
	while ((n = nextNumber()) >= 0) {
		if (n == 0) {
			continue;
		}
		overrideS88(n, false, false);
	}
}

void cmdSetSensors() {
	int n;
	while ((n = nextNumber()) >= 0) {
		if (n == 0) {
			continue;
		}
		int s = nextNumber();
		if (s < 0) {
			Serial.println(F("Syntax error"));
			return;
		}
		overrideS88(n, true, s > 0);
	}
}

void cmdPrintSensors() {
	s88Status();
}

boolean s88MonitorActive = false;

void s88MonitorCallback(char c) {
	if (c == 'q' || c == 'Q') {
		s88MonitorActive = false;
		charModeCallback = NULL;
		Serial.println();
		Serial.println();
		resetTerminal();
	}
}

void s88MonitorDoPrint() {
	Serial.print((char)0x0d);
	for (byte i = 0; i < sizeof(s88Sensorstates); i++) {
		byte x = s88Sensorstates[i];
		if (x < 0x10) {
			Serial.print('0');
		}
		Serial.print(x, HEX); Serial.print(' ');
	}
}

void s88MonitorPrint() {
	if (!s88BusChanged || !s88MonitorActive) {
		return;
	}
	s88BusChanged = false;
	s88MonitorDoPrint();
}

void cmdMonitorS88() {
	s88MonitorActive = true;
	charModeCallback = &s88MonitorCallback;

	Serial.println(F("                                  100                                   200                     "));
	Serial.println(F("00 08 16 24 32 40 48 56 64 72 80 96 04 12 20 28 36 44 52 60 68 76 84 92 00 08 16 24 32 40 48 56"));
	for (byte i = 0; i < sizeof(s88Sensorstates); i++) Serial.print(F("---")); Serial.println();
	s88MonitorDoPrint();
}


void setupS88Support() {
	registerLineCommand("SEN", &cmdSetSensors);
	registerLineCommand("RLS", &cmdReleaseSensors);
	registerLineCommand("S88", &cmdPrintSensors);
	registerLineCommand("S8M", &cmdMonitorS88);

	pinMode(LOAD_INT_0, INPUT_PULLUP) ;
	attachInterrupt(digitalPinToInterrupt(LOAD_INT_0), s88LoadInt, RISING);

	pinMode(CLOCK_INT_1, INPUT_PULLUP) ;
	attachInterrupt(digitalPinToInterrupt(CLOCK_INT_1), s88ClockInt, RISING) ;

	pinMode(DATA_IN, INPUT) ;

	pinMode(DATA_OUT, OUTPUT) ;
	digitalWrite(DATA_OUT, LOW) ;
}

boolean s88ModuleHandler(ModuleCmd cmd) {
  switch (cmd) {
    case initialize:
    	setupS88Support();
      break;
    case eepromLoad:
      return loadEEPROMSensors();
    case eepromSave:
      saveEEPROMSensors();
      break;
    case status:
		s88Status();
		break;
    case dump:
      break;
    case reset:
      resetAllSensors();
      break;
    case periodic:
    	s88InLoop();
    	s88MonitorPrint();
    	break;
  }
  return true;
}

ModuleChain s88Module("S88", 1, &s88ModuleHandler);


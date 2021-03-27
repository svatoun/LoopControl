/*
 * Loops.cpp
 *
 *  Created on: Mar 13, 2021
 *      Author: sdedic
 */
#include <Arduino.h>
#include "Common.h"
#include "Defs.h"
#include "Debug.h"
#include "Loops.h"
#include "S88.h"
#include "Utils.h"

LoopDef	loopDefinitions[maxLoopCount];
LoopState loopStates[maxLoopCount];
int relayPins[maxRelayCount] = {
	RELAY_1, RELAY_2, RELAY_3, RELAY_4
};
boolean relayStates[maxRelayCount] = {};

int loopCount = 0;

long timeDiff(long time) {
	if (time == 0) {
		return 0;
	}
	return millis() - time;
}

boolean defineLoop(int id, const LoopDef& edited) {
	if (id < 0 || id >= maxLoopCount) {
		return false;
	}
	if (id >= maxLoopCount) {
		return false;
	}
	loopDefinitions[id] = edited;
	loopDefinitions[id].active = true;
	loopDefinitions[id].defineSensors();
	freeUnusedSensors();
	return true;
}

String statName(Status s) {
	switch (s) {
	case idle:	return "idle";
	case approach: return "approach";
	case entering: return "entering";
	case moving: return "moving";
	case armed: return "armed";
	case occupied: return "occupied";
	case exiting: return "exiting";
	case exited: return "exited";
	}
	return "N/A";
}

String statName(Status s, Direction d) {
	return statName(s) + (d == left ? "-left" : "-right");
}


int freeUnusedSingleSensor(int sensor) {
	for (int i = 0; i < maxLoopCount; i++) {
		const LoopDef& def = loopDefinitions[i];
		if (!def.active) {
			continue;
		}
		if (def.hasSensor(sensor)) {
			return 0;
		}
	}
	Serial.print(F("Freeing sensor: ")); Serial.println(sensor);
	freeSensor(sensor);
	return 1;
}

int freeUnusedSensors() {
	return forSensors(&freeUnusedSingleSensor);
}

void clearLoop(int id) {
	if (id >= maxLoopCount) {
		return;
	}
	loopDefinitions[id].active = false;
}

void printSensorAndState(const String& name, int sensor, boolean invert, boolean nl) {
	if (invert) {
		Serial.print('~');
	}
	Serial.print(name); Serial.print(F(": "));
	Serial.print(sensor);
	if (sensor == 0) {
		return;
	}
	int v = tryReadS88(sensor);
	char c = (v < 0) ? 'x' : ((v > 0) != invert ? '1' : '0');
	Serial.print(F(" = "));
	if (nl) {
		Serial.println(c);
	} else {
		Serial.print(c);
	}
}

void printSensorAndState(const String& name, int sensor, boolean invert) {
	printSensorAndState(name, sensor, invert, false);
}

boolean Endpoint::stateA() const {
	return readS88(sensorA) != invertA;
}

boolean Endpoint::stateB() const {
	return readS88(sensorB) != invertB;
}

void Endpoint::printState() const {
	printSensorAndState(F("trackA"), sensorA, invertA);
	Serial.print('\t'); printSensorAndState(F("trackB"), sensorB, invertB);
	if (turnout > 0) {
		Serial.print('\t'); printSensorAndState(F("turnout"), turnout, invertTurnout);
	}
	if (useSwitch) {
		Serial.print('\t'); printSensorAndState(F("switch"), switchOrSensor, invertSensor);
	} else {
		Serial.print('\t'); printSensorAndState(F("sensor"), switchOrSensor, invertSensor);
	}
	Serial.print(F("\tRelay=")); Serial.print(triggerState);
	Serial.print(F("\tPrimed: ")); Serial.print(isPrimedEnter());
	Serial.println();
}

void dumpSensor(char type, int sensor, boolean invert) {
	if (sensor <= 0) {
		return;
	}
	Serial.print(':');
	Serial.print(type);
	Serial.print('=');
	if (invert) {
		Serial.print('-');
	}
	Serial.print(sensor);
}

void Endpoint::dump(boolean left) const {
	if (sensorA == 0 &&
		sensorIn == 0 &&
		sensorOut == 0) {
		return;
	}
	Serial.print(F("EPT:"));
	Serial.print(left ? 'L' : 'R');
	if (triggerState) {
		Serial.print('*');
	}
	dumpSensor('A', sensorA, invertA);
	dumpSensor('B', sensorB, invertB);
	dumpSensor('H', shortTrack, invertShortTrack);
	dumpSensor('T', turnout, invertTurnout);
	if ((sensorIn > 0) &&
		(sensorIn == sensorOut)) {
		dumpSensor('S', sensorIn, invertInSensor);
	} else {
		dumpSensor('I', sensorIn, invertInSensor);
		dumpSensor('O', sensorOut, invertOutSensor);
	}
	Serial.println();
};

void LoopCore::printState() const {
	printSensorAndState(F("track"), track, false);
	printSensorAndState(F("\texitLeft"), sensorA, false);
	printSensorAndState(F("\texitRight"), sensorB, false);
	Serial.println();
}

void LoopCore::dump() const {
	if (track == 0) {
		return;
	}
	Serial.print(F("COR:"));
	dumpSensor('T', track, invertTrack);
	Serial.println();
}

void callSensorFunc(int sensor, int& cnt, sensorIteratorFunc fn) {
	if (sensor <= 0) {
		return;
	}
	cnt += fn(sensor);
}

int LoopCore::forSensors(sensorIteratorFunc fn) const {
	int cnt = 0;
	callSensorFunc(sensorA, cnt, fn);
	callSensorFunc(sensorB, cnt, fn);
	callSensorFunc(track, cnt, fn);
	return cnt;
}

boolean LoopCore::hasChanged() const {
	if (track > 0 && s88Changed(track)) {
		return true;
	}
	if (sensorA > 0 && s88Changed(sensorA)) {
		return true;
	}
	if (sensorB > 0 && s88Changed(sensorB)) {
		return true;
	}
	return false;
}

boolean LoopCore::occupied() const {
	if (track > 0) {
		return readS88(track) != invertTrack;
	}
	return false;
}

boolean LoopCore::isPrimed() const {
	if (track > 0) {
		boolean st = readS88(track) != invertTrack;
		if (!st) {
			return false;
		}
	}
	if (sensorA == 0 && sensorB == 0) {
		return true;
	}
	if (sensorA > 0) {
		boolean st = readS88(sensorA) != invertA;
		if (st) {
			return true;
		}
	}
	if (sensorB > 0) {
		boolean st = readS88(sensorB) != invertB;
		if (st) {
			return true;
		}
	}
	return false;
}

int LoopCore::occupiedTrackSensors() const {
	return (track > 0 && readS88(track)) ? 1 : 0;
}


void LoopDef::printState() const {
	Serial.print(F("Active:")); Serial.println(active);
	if (!active) {
		return;
	}
	Serial.print(F("Left:\t")); left.printState();
	Serial.print(F("Right:\t")); right.printState();
	Serial.print(F("Core:\t")); core.printState();
	Serial.println();
}

void LoopDef::dump() const {
	left.dump(true); right.dump(false); core.dump();
	if (relayA > 0 || relayB > 0) {
		Serial.print(F("REL"));
		dumpSensor('A', relayA, false);
		dumpSensor('B', relayA, false);
		Serial.println();
	}
}

int Endpoint::forSensors(sensorIteratorFunc fn) const {
	int cnt = 0;
	callSensorFunc(sensorA, cnt, fn);
	callSensorFunc(sensorB, cnt, fn);
	callSensorFunc(sensorIn, cnt, fn);
	callSensorFunc(sensorOut, cnt, fn);
	callSensorFunc(shortTrack, cnt, fn);
	callSensorFunc(turnout, cnt, fn);
	callSensorFunc(switchOrSensor, cnt, fn);
	return cnt;
}

boolean Endpoint::hasChanged() const {
	if (switchOrSensor > 0 && s88Changed(switchOrSensor)) {
		return true;
	}
	if (sensorA > 0 && s88Changed(sensorA)) {
		return true;
	}
	if (sensorB > 0 && s88Changed(sensorB)) {
		return true;
	}
	return false;
}

/**
 * The endpoint is primed (for enter), if:
 */
boolean Endpoint::isPrimedEnter() const{
	if (!isValidEnter()) {
		return false;
	}
	if (sensorIn > 0) {
		boolean ss = readS88(sensorIn) != invertInSensor;
		return ss;
	}
	if (switchOrSensor > 0 && !useSwitch) {
		// for "primed", the sensor must be active.
		boolean ss = readS88(switchOrSensor) != invertSensor;
		return ss;
	}
	return true;
}

boolean Endpoint::hasTriggerSensors() const {
	return sensorIn > 0 || sensorOut > 0;
}

boolean Endpoint::sensorsActive() const {
	if (sensorIn == 0 || sensorOut == 0) {
		return false;
	}
	boolean active = false;
	if (sensorIn > 0) {
		active = readS88(sensorIn);
	}
	if ((sensorOut > 0) && (sensorIn != sensorOut)) {
		active |= readS88(sensorOut);
	}
	return active;
}

// ONLY valid in
/*
boolean Endpoint::isFullyEntered(LoopState& s) const {
	if (s.status != entering) {
		return false;
	}
	if (isValidEnter()) {
		// still on enter track
		return false;
	}
	if (this != !s.fromEdge()) {
		return false;
	}
	if (sensorIn == 0 || sensorOut == 0) {
		return true;
	}
	boolean active = false;
	if (sensorIn > 0) {
		active = readS88(sensorIn) == invertInSensor;
	}
	if ((sensorOut > 0) && (sensorIn != sensorOut)) {
		active |= (readS88(sensorOut) == invertOutSensor);
	}
	if (active) {
		return false;
	}
	long d = s.dirSensorTime();
	if (d == 0) {
		return false;
	}
	long m = millis();
	d = m - d;
	boolean ret = (d > s.def().sensorThreshold);
	s.resetDirSensor(false);
	return ret;
}
*/

boolean Endpoint::isValidEnter() const {
	if (sensorA > 0 && sensorB > 0) {
		boolean stateA = readS88(sensorA);
		boolean stateB = readS88(sensorB);

		if (shortTrack > 0) {
			boolean state = readS88(shortTrack) != invertShortTrack;
			if (debugLoops) {
				Serial.print(F("Two enter tracks + short track: "));
				Serial.print(shortTrack); Serial.print(F("=")); Serial.println(state);
			}
			if (state) {
				// short track does not depend on divergent enters
				return true;
			}
		}
		if (debugLoops) {
			Serial.print(F("Two enter tracks: "));
			Serial.print(sensorA); Serial.print(F("=")); Serial.print(stateA);
			Serial.print('\t'); Serial.print(sensorB); Serial.print(F("=")); Serial.println(stateB);
		}
		if (useSwitch && switchOrSensor > 0) {
			// Tracks join at turnout whose position is detected. Count the turnout position in:
			boolean switchState = readS88(switchOrSensor) != invertSensor;
			Serial.print(F("Selecting: ")); Serial.println(switchState ? sensorA : sensorB);
			if (switchState) {
				return stateB;
			} else {
				return stateA;
			}
		}
		if (switchOrSensor > 0) {
			// two tracks, with a common sensor. Trigger if either of the tracks is active + the sensor is.
			boolean switchState = readS88(switchOrSensor) != invertSensor;
			Serial.print(F("Sensor: ")); Serial.println(switchState);
			return (stateA || stateB) && (switchState);
		}
		// just two tracks that join, no other trigger.
		return stateA || stateB;
	}
	boolean stateA = readS88(sensorA) != invertA;
	if (debugLoops) {
		Serial.print(F("Single track: ")); Serial.print(sensorA); Serial.print(F("=")); Serial.println(stateA);
	}
	if (shortTrack > 0) {
		boolean state = readS88(shortTrack) != invertShortTrack;
		if (debugLoops) {
			Serial.print(F("Short track: "));
			Serial.print(shortTrack); Serial.print(F("=")); Serial.println(state);
		}
		stateA |= state;
	}
	if (!stateA) {
		return false;
	}
	if (turnout > 0) {
		boolean ss = readS88(turnout) != invertTurnout;
		if (debugLoops) {
			Serial.print(F("Switched: ")); Serial.println(ss);
		}
		if (!ss) {
			return false;
		}
	}
	if (!useSwitch) {
		return true;
	}
	if (switchOrSensor > 0) {
		boolean ss = readS88(switchOrSensor) != invertSensor;
		if (debugLoops) {
			Serial.print(F("Switched: ")); Serial.println(ss);
		}
		return ss;
	} else {
		return true;
	}
}

int Endpoint::selectedExitTrack() const {
	int exitTrack = sensorA;
	int tnt = 0;
	boolean inv = false;
	if (turnout > 0) {
		tnt = turnout;
		inv = invertTurnout;
	}
	if (useSwitch && (switchOrSensor > 0)) {
		tnt = switchOrSensor;
		inv = invertSensor;
	}
	if ((sensorA > 0) && (sensorB > 0) && (tnt > 0)) {
		boolean switchState = readS88(tnt);
		if (switchState != inv) {
			exitTrack = sensorB;
		} else {
			exitTrack = sensorA;
		}
		if (debugLoops) {
			Serial.print(F("Choosing track: ")); Serial.println(exitTrack);
		}
		return exitTrack;
	}
	if (debugLoops) {
		Serial.print(F("Checking track: ")); Serial.println(exitTrack);
	}
	if (sensorB == 0 && (tnt > 0)) {
		boolean switchState = readS88(tnt);
		if (!(switchState != inv)) {
			// turnout in wrong direction
			if (debugLoops) {
				Serial.println(F("Cannot exit: turnout"));
			}
			return 0;
		}
	}
	return exitTrack;
}

boolean Endpoint::occupied() const {
	int trackSensor = selectedExitTrack();
	if (trackSensor < 1) {
		return false;
	}
	if (readS88(trackSensor)) {
		return true;
	}
	return shortTrack > 0 && readS88(shortTrack);
}

boolean Endpoint::changedOccupied(int sensor, boolean occupied) const {
	int trackSensor = selectedExitTrack();
	if (turnout > 0 && (sensor == turnout)) {
		if ((trackSensor < 1) && !occupied) {
			return true;
		}
	}
	if (useSwitch && (sensor == switchOrSensor)) {
		if ((trackSensor < 1) && !occupied) {
			return true;
		}
	}
	if ((trackSensor != sensor) && (shortTrack != sensor)) {
		return false;
	}
	boolean s = readS88(trackSensor);
	if (shortTrack > 0) {
		s |= readS88(shortTrack);
	}
	return s == occupied;
}

boolean Endpoint::isValidExit() const {
	int exitTrack = selectedExitTrack();
	if (exitTrack < 0) {
		// defined, but switched to other direction
		return false;
	}
	if (exitTrack > 0) {
		boolean s = readS88(exitTrack);
		if (shortTrack > 0) {
			s = readS88(shortTrack);
		}
		if (s) {
			if (debugLoops) {
				Serial.println(F("Cannot exit: occupied"));
			}
			return false;
		}
	} else {
		return false;
	}
	if (debugLoops) {
		Serial.println(F("Valid for exit"));
	}
	return true;
}

boolean Endpoint::isPrimedExit() const {
	if (!isValidExit()) {
		return false;
	}
	if (sensorOut > 0) {
		boolean v = readS88(sensorOut) != invertOutSensor;
		if (debugLoops) {
			Serial.print(F("Exit sensor: ")); Serial.println(v);
		}
		return v;
	}
	if (!useSwitch && (switchOrSensor > 0)) {
		boolean v = readS88(switchOrSensor) != invertSensor;
		if (debugLoops) {
			Serial.print(F("Exit sensor: ")); Serial.println(v);
		}
		return v;
	} else {
		if (debugLoops) {
			Serial.println(F("Primed for exit"));
		}
		return true;
	}
}

int Endpoint::occupiedTrackSensors() const {
	int cnt = 0;
	if (sensorA > 0 && readS88(sensorA)) {
		cnt++;
	}
	if (sensorB > 0 && readS88(sensorB)) {
		cnt++;
	}
	if (shortTrack > 0 && readS88(shortTrack)) {
		cnt++;
	}
	return cnt;
}

int LoopDef::forSensors(sensorIteratorFunc fn) const {
	if (!active) {
		return 0;
	}
	return
			left.forSensors(fn) + right.forSensors(fn) + core.forSensors(fn);
}

int defineLoopSensor(int id) {
	return defineSensor(id) ? 0 : 1;
}

void LoopDef::defineSensors() const {
	if (!active) {
		return;
	}
	int errs = forSensors(&defineLoopSensor);
	if (errs > 0) {
		Serial.print(F("Could not define sensors: ")); Serial.println(errs);
	}
}

void resetAllRelays() {
	for (int i = 1; i <= maxRelayCount; i++) {
		pinMode(relayPins[i - 1], OUTPUT);
		switchRelay(i, false);
	}
}

boolean isRelayOn(int rid) {
	if (rid <= 0 || rid > maxRelayCount) {
		return false;
	}
	return relayStates[rid - 1];
}

void switchRelay(int rid, boolean on) {
	if (rid <= 0 || rid > maxRelayCount) {
		return;
	}
	if (debugRelays) {
		Serial.print(F("Setting relay ")); Serial.print(rid); Serial.print(F(" => ")); Serial.println(on);
	}
	digitalWrite(relayPins[rid - 1], (on == relayOnHigh) ? HIGH : LOW);
	relayStates[rid -1] = on;
}

int LoopDef::occupiedTrackSensors() const {
	return left.occupiedTrackSensors() + right.occupiedTrackSensors() + core.occupiedTrackSensors();
}

void LoopDef::printAllStates() {
	for (int i = 0; i < maxLoopCount; i++) {
		if (loopDefinitions[i].active) {
			Serial.print(F("Loop def #")); Serial.print(i + 1); Serial.print(F(" - "));
			loopStates[i].printState();
		}
	}
}

void LoopState::markDirSensor(boolean out) {
	const LoopDef& d = def();
	boolean x = direction == left;
	x = x == out;

	boolean hasS = x ? d.left.hasTriggerSensors() : d.right.hasTriggerSensors();
	if (!hasS) {
		return;
	}
	if (x) {
		leftSensorTime = millis();
		if (debugLoops) {
			Serial.print(F("* Mark left sensor: ")); Serial.println(leftSensorTime);
		}
	} else {
		rightSensorTime = millis();
		if (debugLoops) {
			Serial.print(F("* Mark right sensor: ")); Serial.println(rightSensorTime);
		}
	}
}

void LoopState::printState() const {
	if (!def().active) {
		return;
	}
	const LoopDef& d = def();
	Serial.print(F("Status: ")); Serial.print(statName(status));
	Serial.print(F("\tDirection:")); Serial.println(direction == left ? F("Left") : F("Right"));
	Serial.print(F("RelayA: ")); Serial.print(d.relayA); Serial.print(F("\tRelayB: ")); Serial.print(d.relayB);
	def().printState();
}

void loopSensorCallback(int sensor, boolean state) {
	for (int i = 0; i < maxLoopCount; i++) {
		const LoopDef& def = loopDefinitions[i];
		if (!def.hasSensor(sensor)) {
			continue;
		}
		LoopState &state = loopStates[i];
	}
}

void processSensorTriggers(int sensor, boolean state) {
	for (int i = 0; i < maxLoopCount; i++) {
		LoopState &st = loopStates[i];
		LoopDef &def = loopDefinitions[i];

		if (debugLoops) {
			if (def.hasSensor(sensor)) {
				Serial.print(F("Triggering loop #")); Serial.print(i + 1);
				Serial.println(F(" Initial state"));
				st.printState();
			}
		}
		st.processChange(sensor, state);
		if (debugLoops) {
			if (def.hasSensor(sensor)) {
				Serial.println(F("Loop processed - new state:"));
				st.printState();
			}
		}
	}
}

void resetLoops() {
	Serial.println("Clearing all loops");
	for (int i = 0; i < maxLoopCount; i++) {
		loopDefinitions[i] = LoopDef();
		loopStates[i] = LoopState();
	}
	resetAllRelays();
}

void eepromSaveLoops() {
	eeBlockWrite(0xca, eepromLoopDefs, &loopDefinitions[0], sizeof(loopDefinitions));
}

boolean eepromLoadLoops() {
	if (!eeBlockRead(0xca, eepromLoopDefs, &loopDefinitions[0], sizeof(loopDefinitions))) {
		Serial.println(F("Loop definitions corrupted, resetting"));
		resetLoops();
	}
	for (int i = 0; i < maxLoopCount; i++) {
		const LoopDef& def = loopDefinitions[i];
		def.defineSensors();
		loopStates[i] = LoopState();
	}
	resetAllRelays();
	return true;
}

void relayStatus() {
	for (int i = 0; i < maxRelayCount; i++) {
		Serial.print(F("Relay #")); Serial.print(i + 1); Serial.print(F(" = "));
		Serial.println(isRelayOn(i + 1));
	}
}

boolean periodicTriggers() {
	// now process zero senors that have timed out:
	long m = millis();
	for (int i = 0; i < maxLoopCount; i++) {
		LoopState &st = loopStates[i];
		LoopDef &def = loopDefinitions[i];
		if (st.rightSensorTime > 0 && def.right.hasTriggerSensors()) {
			long d = m - st.rightSensorTime;
			if (debugLoops) {
				Serial.print(F("Loop #")); Serial.print(i + 1);
				Serial.print(F(" Right sensor timeout "));
				Serial.println(d);
			}
			if (d > def.sensorTimeout) {
				int s = def.right.sensorOut;
				if (s == 0) {
					s = def.right.sensorIn;
				}
				st.processChange(s, false);
			}
		}
		if (st.leftSensorTime > 0 && def.left.hasTriggerSensors()) {
			long d = m - st.leftSensorTime;
			if (debugLoops) {
				Serial.print(F("Loop #")); Serial.print(i + 1);
				Serial.print(F(" Left sensor timeout "));
				Serial.println(d);
			}
			if (d > def.sensorTimeout) {
				int s = def.left.sensorOut;
				if (s == 0) {
					s = def.left.sensorIn;
				}
				st.processChange(s, false);
			}
		}
	}
	return true;
}

boolean loopsHandler(ModuleCmd cmd) {
	switch (cmd) {
	case initialize:
		sensorCallback = &processSensorTriggers;
		break;
	case eepromLoad:
		return eepromLoadLoops();

	case eepromSave:
		eepromSaveLoops();
		break;
	case reset:
		resetLoops();
		break;
	case status:
		LoopDef::printAllStates();
		relayStatus();
		break;
	case periodic:
		periodicTriggers();
		break;
	}
	return true;
}

ModuleChain loopChain("Loops", 3, &loopsHandler);


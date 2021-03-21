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
	if (useSwitch) {
		Serial.print('\t'); printSensorAndState(F("switch"), switchOrSensor, invertSensor);
	} else {
		Serial.print('\t'); printSensorAndState(F("sensor"), switchOrSensor, invertSensor);
	}
	Serial.print(F("\tPrimed: ")); Serial.print(isPrimedEnter());
	Serial.println();
}

void LoopCore::printState() const {
	printSensorAndState(F("track"), track, false);
	printSensorAndState(F("\texitLeft"), sensorA, false);
	printSensorAndState(F("\texitRight"), sensorB, false);
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

int Endpoint::forSensors(sensorIteratorFunc fn) const {
	int cnt = 0;
	callSensorFunc(sensorA, cnt, fn);
	callSensorFunc(sensorB, cnt, fn);
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
	if (switchOrSensor > 0 && !useSwitch) {
		// for "primed", the sensor must be active.
		boolean ss = readS88(switchOrSensor) != invertSensor;
		return ss;
	}
	return true;
}

boolean Endpoint::isValidEnter() const {
	if (sensorA > 0 && sensorB > 0) {
		boolean stateA = readS88(sensorA);
		boolean stateB = readS88(sensorB);

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
	if (!stateA) {
		return false;
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
	if ((sensorA > 0) && (sensorB > 0) && useSwitch && (switchOrSensor > 0)) {
		boolean switchState = readS88(switchOrSensor);
		if (switchState != invertSensor) {
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
	if (sensorB == 0 && useSwitch && (switchOrSensor > 0)) {
		boolean switchState = readS88(switchOrSensor);
		if (!(switchState != invertSensor)) {
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
	return readS88(trackSensor);
}

boolean Endpoint::changedOccupied(int sensor, boolean occupied) const {
	int trackSensor = selectedExitTrack();
	if (useSwitch && (sensor == switchOrSensor)) {
		if ((trackSensor < 1) && !occupied) {
			return true;
		}
	}
	if (trackSensor != sensor) {
		return false;
	}
	return readS88(trackSensor) == occupied;
}

boolean Endpoint::isValidExit() const {
	int exitTrack = selectedExitTrack();
	if (exitTrack > 0) {
		if (readS88(exitTrack)) {
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
	return cnt;
}

int LoopDef::forSensors(sensorIteratorFunc fn) const {
	if (!active) {
		return 0;
	}
	return left.forSensors(fn) + right.forSensors(fn) + core.forSensors(fn);
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
		Serial.print(F("Loop def #")); Serial.print(i + 1); Serial.print(F(" - "));
		loopStates[i].printState();
	}
}

void LoopState::printState() const {
	if (!def().active) {
		return;
	}
	Serial.print(F("Status: ")); Serial.print(statName(status));
	Serial.print(F("\tDirection:")); Serial.println(direction == left ? F("Left") : F("Right"));
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

void loopsInLoop() {
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
		break;
	}
	return true;
}

ModuleChain loopChain("Loops", 3, &loopsHandler);


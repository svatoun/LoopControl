/*
 * StationLoopTest.cpp
 *
 *  Created on: Mar 22, 2021
 *      Author: sdedic
 */

#include <Arduino.h>

#include "../Common.h"
#include "../S88.h"
#include "../Loops.h"
#include "../Debug.h"

const int s88ApproachLeft = 1;
const int s88ApproachRight = 4;
const int s88Turnout = 3;
const int s88Loop = 2;

/**
 * The setup:
 *
 * * approach track on the left = 1, terminated by turnout 3.
 * * turnout 3 straight leads to the station
 * * station exit track 2 = approach track on the right
 * * loop section = 2
 * * the loop extends from right of the station to the thrown endpoint of turnout 2
 *
 * If the train goes from the station to the right (4), it enters the loop (2) from the right, going to the left across thrown turnout (3) to
 * free track (1)
 * If the train goes from the left (1), it diverts at thrown turnout (3) to the loop (2), and exits to the right station's entry (4).
 * If the train goes from the left (1) through straight turnout (3), it should not interfere with the loop at all.9
 */
class Station {
	const LoopDef& d = loopDefinitions[0];
	const LoopState& s = loopStates[0];
public:
	Station();
	~Station();

	static boolean commandTest(ModuleCmd cmd);

	void setup();

	void testGoThroughLeft();
	void testGoThroughRight();
	void testRetractFromEntryLeft();
	void testRetractFromEntryRight();
	void testReverseFromMiddleLeft();
	void testReverseFromMiddleRight();

	void testFromRightBackAndForth();
	void testOutageInMiddle();
	void testInterferingTrain();
};

Station::Station() {
	setup();
}

Station::~Station() {
	Serial.println("Station teardown");
	ModuleChain::invokeAll(reset);
}

void Station::setup() {
	Serial.println("Station setup");
	LoopDef def;
	def.left.relay = 1;

	def.left.sensorA = 1;
	def.left.turnout = 3;
	def.left.triggerState = true;

	def.right.sensorA = 4;

	def.core.trackA = 2;

	defineLoop(0, def);
	tick();
	assert(F("Loop is initally idle"), s.status == idle);
	ModuleChain::invokeAll(status);
	debugPrintSeparator();
}

boolean Station::commandTest(ModuleCmd cmd) {
	if (cmd != test) {
		return false;
	}
	ModuleChain::invokeAll(reset);

	Station().testGoThroughLeft();
	Station().testGoThroughRight();
	Station().testRetractFromEntryLeft();
	Station().testRetractFromEntryRight();
	Station().testReverseFromMiddleLeft();

	Station().testReverseFromMiddleRight();
	Station().testFromRightBackAndForth();
	Station().testOutageInMiddle();
	/*
	Station().testInterferingTrain();
	*/
	return true;
}


void Station::testGoThroughLeft() {
	Serial.println(F("Station: go through left"));
	overrideS88(s88Turnout, true, true);
	overrideS88(s88ApproachLeft, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88ApproachLeft, true, false);
	tick();
	assert(F("immediately armed"), s.status == armed);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachRight, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited"), s.status == exited);
	assert(F("relay OFF exited"), !isRelayOn(1));

	overrideS88(s88ApproachRight, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

void Station::testGoThroughRight() {
	Serial.println(F("Station: go through right"));

	overrideS88(s88Turnout, true, true);
	tick();
	assert(F("Still idle"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachRight, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("left"), s.direction == left);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	// not fully in
	assert(F("relay still OFF"), !isRelayOn(1));

	overrideS88(s88ApproachRight, true, false);
	tick();
	assert(F("armed"), s.status == armed);
	assert(F("relay ON 1"), isRelayOn(1));

	overrideS88(s88Turnout, true, false);
	tick();
	assert(F("not armed"), s.status == moving);
	assert(F("relay OFF after turnout"), !isRelayOn(1));

	overrideS88(s88Turnout, true, true);
	tick();
	assert(F("rearmed"), s.status == armed);
	assert(F("relay ON 2"), isRelayOn(1));

	overrideS88(s88ApproachLeft, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay ON 3"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited"), s.status == exited);
	assert(F("relay ON 4"), isRelayOn(1));

	overrideS88(s88ApproachLeft, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

// enter the loop, then go back and away
void Station::testRetractFromEntryLeft() {
	Serial.println(F("Station: retract left"));
	overrideS88(s88Turnout, true, true);
	overrideS88(s88ApproachLeft, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("to right"), s.direction == right);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay ON 1"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited back"), s.status == exited);
	assert(F("relay ON 2"), isRelayOn(1));
	assert(F("to left"), s.direction == left);

	overrideS88(s88ApproachLeft, true, false);
	tick();
	assert(F("idle"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

// enter the loop, then go back and away
void Station::testRetractFromEntryRight() {
	Serial.println(F("Station: retract right"));
	tick();
	assert(F("Still idle"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachRight, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay OFF 1"), !isRelayOn(1));
	assert(F("to left"), s.direction == left);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited back"), s.status == exited);
	assert(F("relay still OFF"), !isRelayOn(1));
	assert(F("to right"), s.direction == right);

	overrideS88(s88ApproachRight, true, false);
	tick();
	assert(F("idle"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

void Station::testReverseFromMiddleLeft() {
	Serial.println(F("Station: left reverse from middle"));
	overrideS88(s88Turnout, true, true);
	overrideS88(s88ApproachLeft, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay ON 1"), isRelayOn(1));

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay ON 2"), isRelayOn(1));

	overrideS88(s88ApproachLeft, true, false);
	tick();
	assert(F("armed"), s.status == armed);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == right);

	// sadly, without a guard sensor, we are DOOMED, since
	// all conditions for exiting the loop are met and the
	// polarity is flipped.
}

void Station::testReverseFromMiddleRight() {
	Serial.println(F("Station: right reverse from middle"));
	overrideS88(s88ApproachRight, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay OFF 1"), !isRelayOn(1));

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF 2"), !isRelayOn(1));
	assert(F("toRight"), s.direction == left);

	overrideS88(s88ApproachRight, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF 3"), !isRelayOn(1));
	assert(F("toRight"), s.direction == left);

	overrideS88(s88Turnout, true, true);
	tick();
	assert(F("armed"), s.status == armed);
	assert(F("relay ON 4"), isRelayOn(1));
	assert(F("toRight"), s.direction == left);

	overrideS88(s88Turnout, true, false);
	tick();
	assert(F("armed"), s.status == moving);
	assert(F("relay OFF 4"), !isRelayOn(1));
	assert(F("toRight"), s.direction == left);

	overrideS88(s88ApproachRight, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay OFF 5"), !isRelayOn(1));
	assert(F("toLeft"), s.direction == right);

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited"), s.status == exited);
}

// Enter the loop, then reverse and go back a little, then go forward to the center
// then back again, then forward to the exiting and back to centre
// and to exited and back again to centre. And finally exit
void Station::testFromRightBackAndForth() {
	Serial.println(F("Station: from right, back and forth"));
	overrideS88(s88ApproachRight, true, true);
	tick();
	assert(F("must approach"), s.status == approach);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachRight, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == left);

	// ***** Move back

	overrideS88(s88ApproachRight, true, true);
	tick();
	assert(F("not armed 2"), s.status == exiting);
	assert(F("relay OFF 2"), !isRelayOn(1));
	assert(F("toRight 2"), s.direction == right);

	// ***** And forth again - to exiting

	overrideS88(s88ApproachRight, true, false);
	tick();
	assert(F("not armed 3"), s.status == moving);
	assert(F("relay OFF 3"), !isRelayOn(1));
	assert(F("toRight 3"), s.direction == left);

	overrideS88(s88Turnout, true, true);
	tick();
	overrideS88(s88ApproachLeft, true, true);
	tick();
	assert(F("exiting 4"), s.status == exiting);
	assert(F("relay ON 4"), isRelayOn(1));
	assert(F("toRight 4"), s.direction == left);

	// cannot go back now, since relay would switch OFF

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited 7"), s.status == exited);
	assert(F("relay ON 7"), isRelayOn(1));
	assert(F("toLeft 7"), s.direction == left);

	// ***** And back
	overrideS88(s88Loop, true, true);
	tick();
	assert(F("entering 8"), s.status == entering);
	assert(F("toLeft 8"), s.direction == right);
	assert(F("Relay ON 8"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited 9"), s.status == exited);
	assert(F("relay ON 9"), isRelayOn(1));
}

void Station::testOutageInMiddle() {
	Serial.println(F("Station: middle outage"));
	overrideS88(s88ApproachRight, true, true);
	tick();
	overrideS88(s88Loop, true, true);
	tick();
	overrideS88(s88ApproachRight, true, false);
	tick();
	LoopDef::printAllStates();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF 1"), !isRelayOn(1));
	assert(F("toRight 1"), s.direction == left);

	// *** All sensors OFF:
	overrideS88(s88ApproachRight, true, false);
	overrideS88(s88Loop, true, false);
	tick();
	assert(F("keep moving"), s.status == moving);
	assert(F("outage detected"), s.outageStart > 0);
	for (int a = 0; a < 30; a++) {
		delay(50);
		tick();
	}
	// *** Recovery
	overrideS88(s88Loop, true, true);
	tick();
	assert(F("keep moving"), s.status == moving);
	assert(F("no outage"), s.outageStart == 0);
	assert(F("toRight"), s.direction == left);

	// continue to left
	overrideS88(s88Turnout, true, true);
	overrideS88(s88ApproachLeft, true, true);
	tick();
	overrideS88(s88Loop, true, false);
	overrideS88(s88ApproachLeft, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == left);
}

void Station::testInterferingTrain() {
	Serial.println(F("Station: interfering train"));
	overrideS88(s88ApproachLeft, true, true);
	tick();
	overrideS88(s88Loop, true, true);
	tick();
	overrideS88(s88ApproachLeft, true, false);
	tick();
	// train comes from outside to approach section
	overrideS88(s88ApproachLeft, true, true);
	tick();
	// switch the turnout
	overrideS88(s88Turnout, true, true);
	tick();

	// get the interfering train out of the way
	overrideS88(s88ApproachLeft, true, false);
	tick();

	// the loop is ready to go, train forward
	assert(F("armed"), s.status == armed);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88ApproachLeft, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited"), s.status == exited);
	assert(F("relay ON exited"), isRelayOn(1));

	overrideS88(s88ApproachLeft, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

#ifdef __test_station

ModuleChain stationTestModule("stationTest", 99, &Station::commandTest);

#endif


#include <Arduino.h>

#include "../Common.h"
#include "../S88.h"
#include "../Loops.h"
#include "../Debug.h"


/**
 * Tests traversal through 'baloon loop':
 *
 * Approach section: 	1
 * Loop section:		2
 * Turnout:				3
 *
 * Turnout straight (= 0) -> Left-to-Right
 * Turnout thrown (=1) -> Right-to-Left
 */
const int s88ApproachCommon = 1;
const int s88Turnout = 3;
const int s88Loop = 2;

class Baloon {
	const LoopDef& d = loopDefinitions[0];
	const LoopState& s = loopStates[0];
public:
	Baloon();
	~Baloon();

	static boolean commandTest(ModuleCmd cmd);

	void setup();

	void testGoThroughLeft();
	void testGoThroughRight();
	void testRetractFromEntryLeft();
	void testRetractFromEntryRight();
	void testReverseFromMiddleLeft();
	void testReverseFromMiddleRight();

	void testFromLeftBackAndForth();
	void testOutageInMiddle();
	void testInterferingTrain();
};

Baloon::Baloon() {
	setup();
}

Baloon::~Baloon() {
	Serial.println("Baloon teardown");
	ModuleChain::invokeAll(reset);
}

void Baloon::setup() {
	Serial.println("Baloon setup");
	LoopDef def;
	def.right.relay = 1;

	def.left.sensorA = 1;
	def.left.turnout = 3;
	def.left.invertTurnout = true;

	def.right.sensorA = 1;
	def.right.turnout = 3;
	def.right.triggerState = true;

	def.core.trackA = 2;

	defineLoop(0, def);
	tick();
	assert(F("Loop is initally idle"), s.status == idle);
	ModuleChain::invokeAll(status);
	debugPrintSeparator();
}

boolean Baloon::commandTest(ModuleCmd cmd) {
	if (cmd != test) {
		return false;
	}
	ModuleChain::invokeAll(reset);

	Baloon().testGoThroughLeft();
	Baloon().testGoThroughRight();
	Baloon().testRetractFromEntryLeft();
	Baloon().testRetractFromEntryRight();
	Baloon().testReverseFromMiddleLeft();
	Baloon().testFromLeftBackAndForth();
	Baloon().testOutageInMiddle();
	Baloon().testInterferingTrain();

	return true;
}

void Baloon::testGoThroughLeft() {
	Serial.println(F("Baloon: go through left"));
	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("must approach"), s.status == approach);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Turnout, true, true);
	tick();
	assert(F("armed"), s.status == armed);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited"), s.status == exited);
	assert(F("relay ON exited"), isRelayOn(1));

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

void Baloon::testGoThroughRight() {
	Serial.println(F("Baloon: go through right"));

	overrideS88(s88Turnout, true, true);
	tick();
	assert(F("Still idle"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Turnout, true, false);
	tick();
	assert(F("armed"), s.status == armed);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exiting"), s.status == exited);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

// enter the loop, then go back and away
void Baloon::testRetractFromEntryLeft() {
	Serial.println(F("Baloon: retract left"));
	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("to right"), s.direction == right);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF"), !isRelayOn(1));


	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited back"), s.status == exited);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("to left"), s.direction == left);

}

// enter the loop, then go back and away
void Baloon::testRetractFromEntryRight() {
	Serial.println(F("Baloon: retract right"));

	overrideS88(s88Turnout, true, true);
	tick();
	assert(F("Still idle"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay ON"), isRelayOn(1));
	assert(F("to left"), s.direction == left);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited back"), s.status == exited);
	assert(F("relay still ON"), isRelayOn(1));
	assert(F("to right"), s.direction == right);

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("idle"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

// Enter fully the loop left-to right, then reverse and return back.
void Baloon::testReverseFromMiddleLeft() {
	Serial.println(F("Baloon: left reverse from middle"));
	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("must approach"), s.status == approach);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == right);

	// ***** Move back

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("moving not armed"), s.status == exiting);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == left);

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("moving not armed"), s.status == exited);
	assert(F("relay OFF"), !isRelayOn(1));
}

// Enter the loop, then reverse and go back a little, then go forward to the center
// then back again, then forward to the exiting and back to centre
// and to exited and back again to centre. And finally exit
void Baloon::testFromLeftBackAndForth() {
	Serial.println(F("Baloon: from left, back and forth"));
	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("must approach"), s.status == approach);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == right);

	// ***** Move back

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("not armed 2"), s.status == exiting);
	assert(F("relay OFF 2"), !isRelayOn(1));
	assert(F("toRight 2"), s.direction == left);

	// ***** And forth again - to exiting

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("not armed 3"), s.status == moving);
	assert(F("relay OFF 3"), !isRelayOn(1));
	assert(F("toRight 3"), s.direction == right);

	overrideS88(s88Turnout, true, true);
	tick();
	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("exiting 4"), s.status == exiting);
	assert(F("relay ON 4"), isRelayOn(1));
	assert(F("toRight 4"), s.direction == right);

	// ***** Move back again
	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("not armed 5"), s.status == moving);
	assert(F("relay ON 5"), isRelayOn(1));
	assert(F("toLeft 5"), s.direction == left);

	// ***** Forward to exited

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("exiting 6"), s.status == exiting);
	assert(F("relay ON 6"), isRelayOn(1));
	assert(F("toRight 6"), s.direction == right);

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited 7"), s.status == exited);
	assert(F("relay ON 7"), isRelayOn(1));
	assert(F("toLeft 7"), s.direction == right);

	// ***** And back
	overrideS88(s88Loop, true, true);
	tick();
	assert(F("entering 8"), s.status == entering);
	assert(F("toLeft 8"), s.direction == left);
	assert(F("Relay ON 8"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited 9"), s.status == exited);
	assert(F("relay ON 9"), isRelayOn(1));
}

void Baloon::testReverseFromMiddleRight() {

}

void Baloon::testOutageInMiddle() {
	Serial.println(F("Baloon: middle outage"));
	overrideS88(s88ApproachCommon, true, true);
	tick();
	overrideS88(s88Loop, true, true);
	tick();
	overrideS88(s88ApproachCommon, true, false);
	tick();
	LoopDef::printAllStates();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == right);

	// *** All sensors OFF:
	overrideS88(s88ApproachCommon, true, false);
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
	assert(F("toRight"), s.direction == right);

	// continue to right
	overrideS88(s88Turnout, true, true);
	overrideS88(s88ApproachCommon, true, true);
	tick();
	overrideS88(s88Loop, true, false);
	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
	assert(F("toRight"), s.direction == right);
}

void Baloon::testInterferingTrain() {
	Serial.println(F("Baloon: interfering train"));
	overrideS88(s88ApproachCommon, true, true);
	tick();
	overrideS88(s88Loop, true, true);
	tick();
	overrideS88(s88ApproachCommon, true, false);
	tick();
	// train comes from outside to approach section
	overrideS88(s88ApproachCommon, true, true);
	tick();
	// switch the turnout
	overrideS88(s88Turnout, true, true);
	tick();

	// get the interfering train out of the way
	overrideS88(s88ApproachCommon, true, false);
	tick();

	// the loop is ready to go, train forward
	assert(F("armed"), s.status == armed);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88ApproachCommon, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited"), s.status == exited);
	assert(F("relay ON exited"), isRelayOn(1));

	overrideS88(s88ApproachCommon, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

#ifdef __test_baloon

ModuleChain baloonTestModule("ballonTest", 99, &Baloon::commandTest);

#endif


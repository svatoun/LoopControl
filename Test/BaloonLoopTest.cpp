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
const int s88Approach = 1;
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
	void Baloon::testRetractFromEntryLeft();
	void Baloon::testRetractFromEntryRight();
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
	def.relayA = 1;

	def.left.sensorA = 1;
	def.left.switchOrSensor = 3;
	def.left.invertSensor = true;	// "left" entry is through straight turnout
	def.left.useSwitch = true;

	def.right.sensorA = 1;
	def.right.switchOrSensor = 3;
	def.right.useSwitch = true;
	def.right.triggerState = true;

	def.core.track = 2;

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

	Baloon().testGoThroughLeft();
	Baloon().testGoThroughRight();
	return true;
}

void Baloon::testGoThroughLeft() {
	Serial.println(F("Baloon: go through left"));
	delay(100);
	overrideS88(s88Approach, true, true);
	tick();
	assert(F("must approach"), s.status == approach);

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Approach, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Turnout, true, true);
	tick();
	assert(F("armed"), s.status == armed);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Approach, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exited"), s.status == exited);
	assert(F("relay ON exited"), isRelayOn(1));

	overrideS88(s88Approach, true, false);
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

	overrideS88(s88Approach, true, true);
	tick();
	assert(F("must approach"), s.status == approach);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Loop, true, true);
	tick();
	assert(F("must enter"), s.status == entering);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Approach, true, false);
	tick();
	assert(F("moving not armed"), s.status == moving);
	assert(F("relay ON"), isRelayOn(1));

	overrideS88(s88Turnout, true, false);
	tick();
	assert(F("armed"), s.status == armed);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Approach, true, true);
	tick();
	assert(F("exiting"), s.status == exiting);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Loop, true, false);
	tick();
	assert(F("exiting"), s.status == exited);
	assert(F("relay OFF"), !isRelayOn(1));

	overrideS88(s88Approach, true, false);
	tick();
	assert(F("exiting"), s.status == idle);
	assert(F("relay OFF"), !isRelayOn(1));
}

// enter the loop, then go back and away
void Baloon::testRetractFromEntryLeft() {

}

// enter the loop, then go back and away
void Baloon::testRetractFromEntryRight() {

}

#ifdef __test_loops

ModuleChain baloonTestModule("ballonTest", 99, &Baloon::commandTest);

#endif

/*
 * Debug.h
 *
 *  Created on: Mar 12, 2021
 *      Author: sdedic
 */

#ifndef DEBUG_H_
#define DEBUG_H_

const int debug = 1;            // set to >0 to activate debug output on the serial console. The collector will use delays 200ms and will print stats each 2s
const int debugSensors = 0;
const int debugLow = 0;
const int debugControl = 1;     // debug control commands
const int debugLed = 0;
const int debugMgmt = 0;
const int debugS88 = 0;
const int debugS88Low = 0;
const int numChannels = 8;      // number of sensors used. Max 5 on Arduino UNO, 8 on Nano.

const int debugLoops = 0;
const int debugTransitions = 0;
const int debugRelays = 1;

#undef __test_s88
#define __test_loop
#define __test_loops

void assert(const char* msg, boolean condition);
void assert(const String& msg, boolean condition);
void tick();
void debugPrintSeparator();

void testS88();
void s88Load(const char* hexString);

typedef void (*testMethod)(void);

class Test {
private:
	static Test* head = NULL;
	Test* next = NULL;
protected:
	Test(testMethod method) {}
};

#endif /* DEBUG_H_ */

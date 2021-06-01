#include <Arduino.h>

#include "Common.h"
#include "Debug.h"
#include "Terminal.h"
#include "Defs.h"

extern long cummulativeS88;
extern long s88IntCount;

void setup() {
	Serial.begin(115200);
	Serial.println(STARTUP_MSG);
	Serial.println("Starting...");

#ifdef __test_s88
	testS88();
#endif

	ModuleChain::invokeAll(initialize);
	setupTerminal();
	ModuleChain::invokeAll(test);

	ModuleChain::invokeAll(eepromLoad);
}

void loop() {
	processTerminal();
	ModuleChain::invokeAll(periodic);
	if ((s88IntCount > 0) && (s88IntCount % 100) == 0) {
		Serial.print("S88 period: "); Serial.println(cummulativeS88 / s88IntCount);
	}
}

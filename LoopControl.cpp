#include <Arduino.h>

#include "Common.h"
#include "Debug.h"
#include "Terminal.h"
#include "Defs.h"

void setup() {
	Serial.begin(115200);
	Serial.println(STARTUP_MSG);
	Serial.println("Starting...");

#ifdef __test_s88
	testS88();
#endif

	ModuleChain::invokeAll(initialize);
	ModuleChain::invokeAll(eepromLoad);

	setupTerminal();

	ModuleChain::invokeAll(test);
}

void loop() {
	processTerminal();

	ModuleChain::invokeAll(periodic);
}

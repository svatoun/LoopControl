/*
 * Debug.cpp
 *
 *  Created on: Mar 21, 2021
 *      Author: sdedic
 */

#include <Arduino.h>
#include "Debug.h"
#include "Common.h"

void debugPrintSeparator() {
	Serial.println();
	Serial.println(F("--------------------------------------------------"));
	Serial.println();
}

void assert(const char* msg, boolean cond) {
	if (cond) {
		return;
	}
	debugPrintSeparator();
	Serial.print(F("*** ERROR: "));
	Serial.println(msg);
	debugPrintSeparator();
	delay(2000);
}

void assert(const String& msg, boolean cond) {
	assert(msg.c_str(), cond);
}

void tick() {
	ModuleChain::invokeAll(periodic);
}

void debugClear() {
	ModuleChain::invokeAll(reset);
}



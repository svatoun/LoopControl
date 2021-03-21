/*
 * S88Test.cpp
 *
 *  Created on: Mar 12, 2021
 *      Author: sdedic
 */

#include <Arduino.h>
#include "../Defs.h"
#include "../S88.h"

extern long lastS88Millis;

void storeS88Bit(int sensorId, int state);

void s88Callback(int sensor, boolean state) {
	Serial.println("");
	Serial.print(F("---> sensor ")); Serial.print(sensor); Serial.print(F(" changed to ")); Serial.println(state);
}

void s88Load(const char* hexString) {
	lastS88Millis = millis();
	int sensor = 1;
	const char* ptr = hexString;
	while (*ptr) {
		ptr++;
	}
	ptr--;
	while (ptr >= hexString) {
		int c = toupper(*ptr) - '0';
		if (c > 9) {
			c -= ('A' - '9');
		}
		if (c > 15) {
			Serial.print(F("Error at index ")); Serial.println(ptr - hexString);
			Serial.print(F("Value: ")); Serial.println(c);
			ptr--;
			continue;
		}
		for (int i = 0; i < 4; i++) {
			storeS88Bit(sensor++, (c & 0x01) > 0 ? 1 : 0);
			c >>= 1;
		}
		ptr--;
	}
	s88InLoop();
}

void testS88() {
	sensorCallback = &s88Callback;
	// initial state
	Sensor::printAll(true);

	// define 3 sensors. One active (5), one dormant (3), one will be flipping
	defineSensor(1);
	defineSensor(3);
	defineSensor(5);
	Serial.println(F("Sensors defined"));
	Sensor::printAll(true);

	s88Load("11");
	Serial.println(F("Sensors loaded"));
	Sensor::printAll(false);

	// sensor #1 flips back to 0
	s88Load("10");
	Sensor::printAll(false);

	// and again to 1
	s88Load("11");
	Sensor::printAll(false);

	delay(100);
	s88Load("10");
	delay(100);
	s88Load("11");
	Serial.println(F("Sensors updated"));
	Sensor::printAll(false);

	delay(200);
	s88Load("11");
	Serial.println(F("All flipped"));
	Sensor::printAll(false);
}


/*
 * Defs.h
 *
 *  Created on: Mar 11, 2021
 *      Author: sdedic
 */

#ifndef DEFS_H_
#define DEFS_H_


#include <EEPROM.h>
#include "Common.h"
#include "Settings.h"
#include "Debug.h"
#include "PinOut.h"

#define STARTUP_MSG "LoopControl (c) Belgarat@klfree.net, v. 1.0, 4/2021"

// The layout of the optoshield assumes, that HIGH on common LED control output will trigger a tranzistor to connect LEDs to GND.
// If experimenting with the Arduino alone on the breadboard, it would be better to trigger LED with LOW, connect LED to D12-GND
// and the phototranzistor to +5V-Ax
const int ledOnValue  = HIGH;
const int ledOffValue  = (ledOnValue == HIGH ? LOW : HIGH);

const int longButtonPress = 500;       // long press, millseconds
const int configButtonPress = 2000;     // config button press, milliseconds
const int resetButtonPress = 10000;     // reset button press, milliseconds
const int calibrationTime = 10000;      // how long is the input measured during calibration
const long configIdleTimeExit = 60000; // after 60 sec of inactivity the configuration closes

const int reportS88Loss = 1;          // will flash LED if S88 CLK signal is not present

const int eepromVersion = 0x00;
const int eepromSensors = 0x02;
const int eepromStates = 0x30;
const int eepromLoopDefs = 0x50;
const int eepromChecksum = 0x100;


extern void (* charModeCallback)(char);

#endif /* DEFS_H_ */

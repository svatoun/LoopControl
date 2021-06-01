/*
 * PinOut.h
 *
 *  Created on: Mar 12, 2021
 *      Author: sdedic
 */

#ifndef PINOUT_H_
#define PINOUT_H_


// ------- S88 interface -----------
const int LOAD_INT_0      = 2 ;        // 2 LOAD 0 int
const int CLOCK_INT_1     = 3 ;        // 3 CLOCK 1 int
const int DATA_IN         = 4 ;        // data in
const int DATA_OUT        = 5 ;        // data out

/**
 * Button "plus" from the control panel; input
 */
const int BUTTON_PLUS     = 6;

/**
 * Button "minus" from the control panel; input
 */
const int BUTTON_MINUS    = 7;

/**
 * Button "next" from the control panel; input
 */
const int BUTTON_NEXT     = 8;

/**
 * Indicator LED from the control panel; output
 */
const int LED_SIGNAL      = 13;       // signal LED

/**
 * Indicator LED on the Arduino's body; output.
 */
const int LED_ACK         = 13;       // ACK LED

/**
 * Relays.
 */
const int RELAY_1		  = 12;
const int RELAY_2		  = 11;
const int RELAY_3		  = 9;
const int RELAY_4		  = 10;

// 2345   9 10 11 12  	6 7 8 A0  vstupy: A1 A2 A3 A4

const boolean relayOnHigh = false;


#endif /* PINOUT_H_ */

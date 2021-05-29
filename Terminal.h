/*
 * Terminal.h
 *
 *  Created on: Mar 16, 2021
 *      Author: sdedic
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

extern char *promptString;
extern void (* charModeCallback)(char);

void resetTerminal();
void setupTerminal();
void processTerminal();
void printPrompt();

#endif /* TERMINAL_H_ */

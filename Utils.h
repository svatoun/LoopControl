/*
 * Utils.h
 *
 *  Created on: Mar 11, 2021
 *      Author: sdedic
 */

#ifndef UTILS_H_
#define UTILS_H_


void makeLedAck(const int *  ledSequence);
void writeEEPROM();
void commandReset();
void resetEEPROM();

void eeBlockWrite(byte magic, int eeaddr, const void* address, int size);
boolean eeBlockRead(byte magic, int eeaddr, void* address, int size);
int eepromWriteInt(int addr, int t, int& checksum);
int eepromWriteByte(int addr, byte t, int& checksum);
int eepromReadByte(int &addr, int& checksum, boolean& allzero);
int eepromReadByte(int &addr, int& checksum, boolean& allzero);
int eepromReadInt(int &addr, int& checksum, boolean& allzero);

void debugPrintSeparator();

extern long lastLedSignalled;

#endif /* UTILS_H_ */

/*
 * Utils.cpp
 *
 *  Created on: Mar 11, 2021
 *      Author: sdedic
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "Defs.h"
#include "Utils.h"

// ========================= ModuleChain ================================

ModuleChain* ModuleChain::head;

ModuleChain::ModuleChain(const char* name, byte aPrirority,  boolean (*h)(ModuleCmd)) : next(NULL), priority(aPrirority) {
  static ModuleChain* __last = NULL;
  if (__last == NULL) {
    head = NULL;
  }
  __last = this;
  handler = h;
  if (head == NULL) {
    head = this;
    return;
  }
  if (head->priority >= aPrirority) {
    next = head;
    head = this;
    return;
  }
  ModuleChain* p = head;
  while (p->priority < aPrirority) {
    if (p->next == NULL) {
      p->next = this;
      return;
    }
    p = p->next;
  }
  if (p->next == NULL) {
    p->next = this;
    return;
  }
  next = p->next;
  p->next = this;
}

boolean ModuleChain::invokeAll(ModuleCmd cmd) {
  ModuleChain*p = head;
  boolean status = true;
  while (p != NULL) {
    if (p->handler != NULL) {
		boolean r = p->handler(cmd);
		status &= r;
    }
    p = p->next;
  }
  return status;
}


// ========================= EEPROM functions ================================
int eepromWriteByte(int addr, byte t, int& checksum) {
    checksum = checksum ^ t;
    EEPROM.update(addr++, (t & 0xff));
    return 0;
}

int eepromWriteInt(int addr, int t, int& checksum) {
    checksum = checksum ^ t;
    EEPROM.update(addr++, (t & 0xff));
    EEPROM.update(addr++, (t >> 8) & 0xff);
    if (debugControl) {
      Serial.print(t & 0xff, HEX); Serial.print((t >> 8) & 0xff, HEX); Serial.print(" ");
    }
    return addr;
}


int eepromReadByte(int &addr, int& checksum, boolean& allzero) {
    int v = EEPROM.read(addr);
    addr ++;
    checksum = checksum ^ v;
    if (v != 0) {
    	allzero = false;
    }
    return v;
}

int eepromReadInt(int &addr, int& checksum, boolean& allzero) {
    int v = EEPROM.read(addr) + (EEPROM.read(addr + 1) << 8);
    addr += 2;
    checksum = checksum ^ v;
    Serial.print(v & 0xff, HEX); Serial.print("-"); Serial.print((v >> 8) & 0xff, HEX); Serial.print(" ");
    if (v != 0) {
      allzero = false;
    }
    Serial.print(F(" = ")); Serial.println(v);
    return v;
}

/**
   Writes a block of data into the EEPROM, followed by a checksum
*/
void eeBlockWrite(byte magic, int eeaddr, const void* address, int size) {
  if (debugControl) {
    Serial.print(F("Writing EEPROM ")); Serial.print(eeaddr, HEX); Serial.print(F(":")); Serial.print(size); Serial.print(F(", source: ")); Serial.println((int)address, HEX);
  }
  const byte *ptr = (const byte*) address;
  byte hash = magic;
  EEPROM.write(eeaddr, magic);
  eeaddr++;
  for (; size > 0; size--) {
    byte b = *ptr;
    EEPROM.write(eeaddr, b);
    ptr++;
    eeaddr++;
    hash = hash ^ b;
  }
  EEPROM.write(eeaddr, hash);
}

/**
   Reads block of data from the EEPROM, but only if the checksum of
   that data is correct.
*/
boolean eeBlockRead(byte magic, int eeaddr, void* address, int size) {
  if (debugControl) {
    Serial.print(F("Reading EEPROM ")); Serial.print(eeaddr, HEX); Serial.print(F(":")); Serial.print(size); Serial.print(F(", dest: ")); Serial.println((int)address, HEX);
  }
  int a = eeaddr;
  byte hash = magic;
  byte x;
  boolean allNull = true;
  x = EEPROM.read(a);
  if (x != magic) {
    if (debugControl) {
      Serial.println(F("No magic header found"));
    }
    return false;
  }
  a++;
  for (int i = 0; i < size; i++, a++) {
    x = EEPROM.read(a);
    if (x != 0) {
      allNull = false;
    }
    hash = hash ^ x;
  }
  x = EEPROM.read(a);
  if (hash != x || allNull) {
    if (debugControl) {
      Serial.println(F("Checksum does not match"));
    }
    return false;
  }

  a = eeaddr + 1;
  byte *ptr = (byte*) address;
  for (int i = 0; i < size; i++, a++) {
    x = EEPROM.read(a);
    *ptr = x;
    ptr++;
  }
  return true;
}

int readEepromByte(int &addr, int& checksum, boolean& allzero) {
    int v = EEPROM.read(addr);
    addr ++;
    checksum = checksum ^ v;
    return v;
}

int readEepromInt(int &addr, int& checksum, boolean& allzero) {
    int v = EEPROM.read(addr) + (EEPROM.read(addr + 1) << 8);
    addr += 2;
    checksum = checksum ^ v;
    Serial.print(v & 0xff, HEX); Serial.print("-"); Serial.print((v >> 8) & 0xff, HEX); Serial.print(" ");
    if (v != 0) {
      allzero = false;
    }
    Serial.print(F(" = ")); Serial.println(v);
    return v;
}

void resetEEPROM() {
	Serial.println(F("Resetting EEPROM"));
	ModuleChain::invokeAll(reset);
	ModuleChain::invokeAll(eepromSave);
	makeLedAck(&blinkReset[0]);
}

const int* blinkPtr = NULL;
long blinkLastMillis;
byte pos = 0;
boolean ackLedState = false;
int pulseCount = 0;
long lastLedSignalled = 0;

void makeLedAck(const int *  ledSequence) {
    blinkPtr = ledSequence;
    pos = 0;
    ackLedState = 1;
    blinkLastMillis = millis();
    lastLedSignalled = blinkLastMillis;
    digitalWrite(LED_ACK, HIGH);
    if (debugLed) {
      Serial.print(F("LED ACK start: ")); Serial.println(blinkPtr[pos]);
    }
}

boolean isAckRunning() {
  return blinkPtr != NULL;
}

void handleAckLed() {
  if (blinkPtr == NULL) {
    return;
  }
  long t = millis();
  long l = t - blinkLastMillis;
  lastLedSignalled = t;
  if (l < blinkPtr[pos]) {
    return;
  }
  blinkLastMillis = t;
  pos++;
  if (debugLed) {
    Serial.print(F("Next ACK time: ")); Serial.println(blinkPtr[pos]);
  }
  if (blinkPtr[pos] == 0) {
    ackLedState = 0;
    digitalWrite(LED_ACK, LOW);
    if (pulseCount > 0) {
      pulseCount--;
      makeLedAck(&blinkShort[0]);
    } else {
      if (debugLed) {
        Serial.println("ACK done");
      }
      blinkPtr = NULL;
      blinkLastMillis = -1;
    }
    return;
  }
  ackLedState = !ackLedState;
  digitalWrite(LED_ACK, ackLedState ? HIGH : LOW);
}

void commandClear() {
  resetEEPROM();
  delay(500);
  commandReset();
}

void commandReset() {
  void (*func)() = 0;
  func();
}

void commandStatus() {
	Serial.println(F("Status report"));
	ModuleChain::invokeAll(status);
}

void commandSave() {
	Serial.println(F("Saving to EEPROM"));
	ModuleChain::invokeAll(eepromSave);
}

boolean utilsModuleHandler(ModuleCmd cmd) {
	switch (cmd) {
	case initialize:
		registerLineCommand("INF", &commandStatus);
		registerLineCommand("RST", &commandReset);
		registerLineCommand("CLR", &commandClear);
		registerLineCommand("SAV", &commandSave);
		break;
	}
	return true;
}

ModuleChain utils("Utils", 20, &utilsModuleHandler);


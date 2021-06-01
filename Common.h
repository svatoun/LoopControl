#ifndef __common_h__
#define __common_h__

#include <Arduino.h>

struct LineCommand {
  const char* cmd;
  void (*handler)();

  LineCommand() : cmd(NULL), handler(NULL) {}
};

void registerLineCommand(const char* cmd, void (*aHandler)());


enum ModuleCmd {
  initialize,
  eepromLoad,
  beforeLoop,
  eepromSave,
  status,
  reset,
  dump,
  periodic,
  test
};

struct ModuleChain {
  static ModuleChain* head;

  byte priority;
  ModuleChain *next;
  boolean (*handler)(ModuleCmd);

  ModuleChain(const char* n, byte priority, boolean (*h)(ModuleCmd));

  static boolean invokeAll(ModuleCmd cmd);
};

extern char printBuffer[];

 __inline __attribute__((always_inline)) char* append(char* &ptr, char c) {
  *(ptr++) = c;
  return ptr;
}

 __inline __attribute__((noinline)) char* initPrintBuffer() {
  printBuffer[0] = 0;
  return printBuffer;
}

__inline __attribute__((noinline)) char *printNumber(char *out, int no, int base) {
  itoa(no, out, base);
  return out + strlen(out);
}

extern char *inputPos;
extern char *inputEnd;


int nextNumber();
void setupTerminal();

void commandReset();
void commandClear();

#endif


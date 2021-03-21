
#include "Common.h"
#include "Defs.h"
#include "Utils.h"
#include "Loops.h"
#include "S88.h"
#include "Terminal.h"

int editedLoopId = -1;
LoopDef	editedLoop;

char defPrompt[8];

int countSensors(int s) {
	return 1;
}

void commandLoop() {
	if (editedLoopId != -1) {
		Serial.println(F("Still editing"));
		return;
	}
	int loop = nextNumber();
	if (loop < 0) {
		for (int i = 0; i < maxLoopCount; i++) {
			const LoopDef& def = loopDefinitions[i];
			if (!def.active) {
				loop = i;
				break;
			}
		}
		if (loop == -1) {
			Serial.println(F("No loop slots."));
			return;
		} else {
			Serial.print(F("Editing loop ")); Serial.println(loop + 1);
		}
	} else {
		if (loop > maxLoopCount) {
			Serial.println(F("Invalid loop ID"));
			return;
		}
		loop--;
	}
	editedLoopId = loop;
	editedLoop = loopDefinitions[loop];

	Serial.println(F("Initial state: "));
	editedLoop.printState();

	sprintf(defPrompt, "L-%d: ", (editedLoopId + 1));
	promptString = defPrompt;
}

void commandEndpoint() {
	if (editedLoopId == -1) {
		Serial.print(F("Not editing."));
		return;
	}
	Endpoint* ptr = NULL;
	switch (*inputPos) {
		case 'l': case 'L': case '1': case 'a': case 'A':
			ptr = &(editedLoop.left);
			break;
		case 'r': case 'R': case '2': case 'b': case 'B':
			ptr = &(editedLoop.right);
			break;
		default:
			Serial.println("Invalid endpoint");
			return;
	}
	inputPos++;
	if (*inputPos == '-' || *inputPos == '~' || *inputPos == '!') {
		ptr->triggerState = false;
	} else if (*inputPos == '+') {
		ptr->triggerState = true;
	} else if (*inputPos == ':') {
		ptr->triggerState = false;
		inputPos--;
	} else {
		Serial.println(F("Trigger state not +/-"));
		return;
	}
	inputPos++;
	if (*inputPos != ':') {
		Serial.println(F("Syntax error"));
		return;
	}
	inputPos++;
	do {
		char c = *inputPos;
		switch (c) {
		case 'a': case 'A':
		case 'b': case 'B':
		case 's': case 'S':
		case 't': case 'T':
			break;
		default:
			Serial.println(F("Invalid sensor (a-b-s-t)"));
			return;
		}
		inputPos++;
		boolean invert = false;
		if (*inputPos != '=') {
			Serial.println(F("Syntax error"));
			return;
		}
		inputPos++;
		if (*inputPos == '-' || *inputPos == '~' || *inputPos == '!') {
			invert = true;
			inputPos++;
		} else if (*inputPos == '+') {
			inputPos++;
		}
		int sno = nextNumber();
		if (sno < 1 || sno > maxSensorCount) {
			Serial.println("Invalid sensor number");
			return;
		}
		switch (c) {
		case 'a': case 'A':
			ptr->sensorA = sno;
			ptr->invertA = invert;
			break;
		case 'b': case 'B':
			ptr->sensorB = sno;
			ptr->invertB = invert;
			break;
		case 's': case 'S':
			ptr->switchOrSensor = sno;
			ptr->invertSensor = invert;
			ptr->useSwitch = false;
			break;
		case 't': case 'T':
			ptr->switchOrSensor = sno;
			ptr->invertSensor = invert;
			ptr->useSwitch = true;
			break;
		default:
			Serial.println(F("Error."));
			return;
		}
	} while (*inputPos);
	Serial.println(F("New endpoint def:"));
	ptr->printState();
}

void commandCore() {
	LoopCore* ptr = &editedLoop.core;
	do {
		char c = *inputPos;
		switch (c) {
		case 'a': case 'A':
		case 'b': case 'B':
		case 't': case 'T':
			break;
		default:
			Serial.println(F("Invalid sensor (a-b-t)"));
			return;
		}
		inputPos++;
		boolean invert = false;
		if (*inputPos != '=') {
			Serial.println(F("Syntax error"));
			return;
		}
		inputPos++;
		if (*inputPos == '-' || *inputPos == '~' || *inputPos == '!') {
			invert = true;
			inputPos++;
		} else if (*inputPos == '+') {
			inputPos++;
		}
		int sno = nextNumber();
		if (sno < 1 || sno > maxSensorCount) {
			Serial.println("Invalid sensor number");
			return;
		}
		switch (c) {
		case 'a': case 'A':
			ptr->sensorA = sno;
			ptr->invertA = invert;
			break;
		case 'b': case 'B':
			ptr->sensorB = sno;
			ptr->invertB = invert;
			break;
		case 't': case 'T':
			ptr->track= sno;
			ptr->invertTrack = invert;
			break;
		default:
			Serial.println(F("Error."));
			return;
		}
	} while (*inputPos);
	Serial.println(F("New core def:"));
	ptr->printState();
}

void commandCancel() {
	if (editedLoopId == -1) {
		Serial.print(F("No edit to cancel."));
		return;
	}
	const LoopDef& orig = loopDefinitions[editedLoopId];
	editedLoopId = -1;
	freeUnusedSensors();
	if (orig.active) {
		Serial.println(F("Cancelled. Original state:"));
		orig.printState();
	} else {
		Serial.println(F("Discarded."));
	}
	promptString = NULL;
}

void commandFinish() {
	if (editedLoopId == -1) {
		Serial.print(F("No edit to finish."));
		return;
	}
	if (editedLoopId >= maxLoopCount) {
		editedLoopId = -1;
		freeUnusedSensors();
		return;
	}
	editedLoop.active = true;
	int sc = editedLoop.forSensors(&countSensors);
	if (sc == 0) {
		Serial.print(F("Invalid, no sensors"));
		commandCancel();
		return;
	}
	if (editedLoop.relayA > 0 || editedLoop.relayB > 0) {
		Serial.print(F("Invalid, no relays"));
		commandCancel();
		return;
	}
	defineLoop(editedLoopId, editedLoop);
	editedLoopId = -1;
	Serial.println(F("Finished, changed state:"));
	editedLoop.printState();
	promptString = NULL;
}


void initCommands() {
  registerLineCommand("DEF", &commandLoop);
  registerLineCommand("EPT", &commandEndpoint);
  registerLineCommand("COR", &commandCore);
  registerLineCommand("CAN", &commandCancel);
  registerLineCommand("FIN", &commandFinish);
}

boolean commandHandler(ModuleCmd cmd) {
	switch (cmd) {
	case initialize:
		initCommands();
		break;
	}
	return true;
}

ModuleChain commandChain("Cmds", 3, &commandHandler);

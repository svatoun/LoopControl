
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
	if (!editedLoop.active) {
		editedLoop = LoopDef();
	}

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
	} else if ((*inputPos == '+') || (*inputPos == '*')) {
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
		case 'a': case 'A':	// sensor 'A'
		case 'b': case 'B': // sensor 'B'
		case 'i': case 'I': // IN sensor
		case 'o': case 'O': // OUT sensor
		case 's': case 'S': // IN+OUT sensor
		case 'h': case 'H': // sHort track, but 's' is already taken
		case 't': case 'T': // Turnout
			break;
		default:
			Serial.println(F("Sensor not in [abhiost]"));
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
		if (sno < 0 || sno > maxSensorId) {
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
		case 'h': case 'H':
			ptr->shortTrack = sno;
			ptr->invertShortTrack = invert;
			break;
		case 'i':
			ptr->sensorIn = sno;
			ptr->invertInSensor = invert;
			break;
		case 'o':
			ptr->sensorIn = sno;
			ptr->invertOutSensor = invert;
			break;
		case 's': case 'S':
			ptr->sensorIn = sno;
			ptr->invertInSensor = invert;
			ptr->sensorIn = sno;
			ptr->invertOutSensor = invert;
			break;
		case 't': case 'T':
			ptr->turnout = sno;
			ptr->invertTurnout = invert;
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
		if (sno < 0 || sno > maxSensorId) {
			Serial.println("Invalid sensor number");
			return;
		}
		switch (c) {
		case 'a': case 'A':
			ptr->trackA = sno;
			ptr->invertA = invert;
			break;
		case 'b': case 'B':
			ptr->trackB = sno;
			ptr->invertB = invert;
			break;
		default:
			Serial.println(F("Error."));
			return;
		}
	} while (*inputPos);
	Serial.println(F("New core def:"));
	ptr->printState();
}

void commandRelay() {
	do {
		char c = *inputPos;
		Endpoint* e;
		Serial.println(c);
		switch (c) {
		case 'a': case 'A': case 'L': case 'l':
			e = &editedLoop.left;
			break;
		case 'b': case 'B': case 'R': case 'r':
			e = &editedLoop.right;
			break;
		default:
			Serial.println(F("Relay not in [ab]"));
			return;
		}
		inputPos++;
		if (*inputPos != '=') {
			Serial.println(F("Syntax error"));
			return;
		}
		inputPos++;
		int sno = nextNumber();
		if (sno < 0 || sno > maxRelayCount) {
			Serial.println("Invalid relay");
			return;
		}
		e->relay = sno;
		e->relayTriggerState = true;
		e->relayOffState = false;
		if (*inputPos) {
			switch (*inputPos) {
			case '1': case '+': case 'H':
				e->relayTriggerState = true;
				break;
			case '0': case '-': case 'L':
				e->relayTriggerState = false;
				break;
			default:
				Serial.println("Invalid relay state");
				return;
			}
			inputPos++;
			switch (*inputPos) {
			case '1': case '+': case 'H':
				e->relayOffState = true;
				break;
			case '0': case '-': case 'L':
				e->relayOffState = false;
				break;
			case 0:
				break;
			default:
				Serial.println("Invalid relay state");
				return;
			}
			inputPos++;
		}
	} while (*inputPos);
	Serial.println(F("Relay defined"));
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
	if (editedLoop.left.relay== 0 && editedLoop.right.relay== 0) {
		Serial.print(F("Invalid, no relays"));
		commandCancel();
		return;
	}
	if ((editedLoop.left.relay == editedLoop.right.relay) && (editedLoop.left.relayTriggerState == editedLoop.right.relayTriggerState)) {
		Serial.print(F("Invalid, relay doesn't change"));
		commandCancel();
		return;
	}
	defineLoop(editedLoopId, editedLoop);
	editedLoopId = -1;
	Serial.println(F("Finished, changed state:"));
	editedLoop.printState();
	promptString = NULL;
}

void commandDump() {
	for (int i = 0; i < maxLoopCount; i++) {
		const LoopDef& d = loopDefinitions[i];
		if (!d.active) {
			continue;
		}
		Serial.print(F("DEF:")); Serial.println(i + 1);
		d.dump();
		Serial.println(F("FIN"));
	}
}

void commandDelete() {
	int ln = nextNumber();
	if (ln < 0) {
		Serial.println(F("Syntax error"));
		return;
	}
	if ((ln <  1) || (ln > maxLoopCount)) {
		Serial.println(F("Invalid number"));
	}
	loopDefinitions[ln - 1] = LoopDef();
	loopStates[ln - 1] = LoopState();
}

boolean monitorActive = false;

void commandMonitor() {
	monitorActive = true;
}

/**
 * ...|..|...
 * ..I|..|...
 * ++I|..|...
 *
 * Monitor state of a loop:
 * ABTIOS-- --ABTIOS
 */

void monitorPrint() {
	if (!monitorActive) {
		return;
	}
	for (int i = 0; i < maxLoopCount; i++) {
		const LoopDef& d = loopDefinitions[i];
		const LoopState& s = loopStates[i];
		if (i > 0) {
			Serial.print('\t');
		}
		Serial.print(i + 1);
		s.monitorPrint();
	}
	Serial.print((char)0x0d);
}


void initCommands() {
  registerLineCommand("DEF", &commandLoop);
  registerLineCommand("EPT", &commandEndpoint);
  registerLineCommand("COR", &commandCore);
  registerLineCommand("REL", &commandRelay);
  registerLineCommand("CAN", &commandCancel);
  registerLineCommand("FIN", &commandFinish);
  registerLineCommand("DEL", &commandDelete);
  registerLineCommand("DMP", &commandDump);
}

boolean commandHandler(ModuleCmd cmd) {
	switch (cmd) {
	case initialize:
		initCommands();
		break;
	case periodic:
		monitorPrint();
		break;
	}
	return true;
}

ModuleChain commandChain("Cmds", 3, &commandHandler);

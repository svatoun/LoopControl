/*
 * LoopStateMachine.cpp
 *
 *  Created on: Mar 20, 2021
 *      Author: sdedic
 */

#include <Arduino.h>
#include "Debug.h"
#include "Loops.h"
#include "S88.h"

/**
 * This is a state automaton for the loop. It works with the following conditions
 *
 * Endpoint::isValidExit - true, if exit is possible at the moment.
 * 	- exit track UNOCCUPIED (incl. track selection based on exit turnout)
 * 	- exit turnout switched to the correct direction
 *
 * Endpoing::isPrimedExit - true, if the train is exiting ATM:
 *  - exit track OCCUPIED (incl. track selection)
 *  - exit turnout switched correctly
 *  - exit sensor fires
 *
 * Endpoint::isValidEnter - true, if enter is possible at the moment
 *  - enter track occupied
 *  - exit turnout switched correctly
 *
 * Endpoint::occupied() - true, if the selected exit track is occupied
 *  - false, if the entry turnout directs to different route than the loop
 *
 * Endpoint::isPrimedEnter - true, if enter is possible at the moment
 *  - enter track occupied
 *  - exit turnout switched correctly
 *  - sensor fires (if defined)
 *
 * The state automaton:
 *
 * idle -> approach -> entering -> moving ->  armed -> exiting -> exited   -> idle
 * 						  |          |          \----\   |           |          |
 * 						  v			 v                \  v           v          v
 * 						exited  -> exiting -> armed -> moving  -> entering -> approach -> idle
 *
 * And "Moving" and "Idle" are auto-transitional states:
 * - if exit conditions are met, Moving transitions to Armed
 * - if other endpoint's enter conditions are met, Idle transitions to Approach.
 *
 */

void LoopState::maybeArm(const Endpoint& via) {
	const LoopDef& d = def();
	if (!d.core.isPrimed()) {
		switchStatus(idle, via);
		return;
	}
	if (!via.isPrimedExit()) {
		if (debugTransitions) {
			Serial.println(F("Exit not ready"));
		}
		return;
	}
	switchStatus(armed, via);
}

void LoopState::processApproach(int sensor, const Endpoint& ep) {
	const LoopDef& d = def();
	if (ep.hasSensor(sensor) && ep.changedOccupied(sensor, false)) {
		if (d.core.isPrimed()) {
			if (debugTransitions) {
				Serial.println(F("Core section jumped to"));
			}
			switchStatus(moving, ep);
			return;
		} else {
			if (debugTransitions) {
				Serial.println(F("Left & core is empty"));
			}
			switchStatus(idle, ep);
		}
	}
	if (d.core.hasSensor(sensor)) {
		if (debugTransitions) {
			Serial.println("Core sensor changed");
			Serial.println(direction == left ? " -> Left" : "-> Right");
			Serial.println(&ep == &d.left ? "Left" : "Right");
		}
		if (ep.occupied() && d.core.isPrimed()) {
			if (debugTransitions) {
				Serial.println(F("Core section partially entered"));
			}
			switchStatus(entering, ep);
		}
	}
}

void LoopState::maybeApproach(Status prevStatus, const Endpoint& from) {
	if (!from.isPrimedEnter()) {
		return;
	}
	direction = directionFrom(from);
	switchStatus(approach, from);
}

void LoopState::switchStatus(Status s, const Endpoint& ep) {
	Status oldStatus = status;

	status = s;
	if (debugTransitions) {
		Serial.print(F("Change status: ")); Serial.println(statName(s));
	}
	const LoopDef& d = def();

	switch (s) {
		case approach:
			switchRelay(d.relayA, fromEdge().triggerState);
			switchRelay(d.relayB, fromEdge().triggerState);
			break;

		case armed:
			switchRelay(d.relayA, toEdge().triggerState);
			switchRelay(d.relayB, toEdge().triggerState);
			break;

		case idle:
			switchRelay(d.relayA, false);
			switchRelay(d.relayB, false);

			maybeApproach(oldStatus, d.opposite(ep));

			break;

		case moving:
			maybeArm(d.opposite(ep));
			break;
	}
}

/**
 * IDLE ->
 *
 * left primed => enteringLeft
 * right primed => enteringRight
 *
 * core appeared (no left or right) => occupied
 */
void LoopState::processIdle(int sensor) {
	const LoopDef& d = def();

	// this is a possible boot-up from power loss or a short
	boolean leftWasPrimed = d.left.hasChanged() && d.left.isPrimedEnter();
	boolean rightWasPrimed = d.right.hasChanged() && d.right.isPrimedEnter();
	boolean coreWasPrimed = d.core.hasChanged() && d.core.isPrimed();

	if (coreWasPrimed) {
		if (leftWasPrimed && rightWasPrimed) {
			if (debugTransitions) {
				Serial.print("Cold boot: occupied, waiting for changes");
			}
			switchStatus(occupied, d.left);
			return;
		}
		if (!(leftWasPrimed || rightWasPrimed)) {
			if (debugTransitions) {
				Serial.print("Cold boot: occupied, direction unknown");
			}
			switchStatus(occupied, d.left);
			return;
		}
		if (leftWasPrimed) {
			direction = left;
			if (debugTransitions) {
				Serial.print("Cold boot: core + left");
			}
			switchStatus(armed, d.left);
			switchStatus(exiting, d.left);
		} else if (rightWasPrimed) {
			direction = right;
			if (debugTransitions) {
				Serial.print("Cold boot: core + right");
			}
			switchStatus(armed, d.right);
			switchStatus(exiting, d.right);
		}
		return;
	}

	if (debugTransitions) {
		Serial.println("Checking left");
	}
	boolean leftReady = d.left.hasSensor(sensor) && d.left.isPrimedEnter();
	if (debugTransitions) {
		Serial.println("Checking right");
	}
	boolean rightReady = d.right.hasSensor(sensor) && d.right.isPrimedEnter();

	if (debugTransitions) {
		Serial.print(F("Left:  ")); d.left.printState();
		Serial.print(F("Right: ")); d.right.printState();
	}

	if (leftReady && rightReady) {
		// no op
		if (debugTransitions) {
			Serial.println(F("Both active => idle"));
		}
		return;
	}
	if (leftReady) {
		direction = right;
		switchStatus(approach, d.left);
	} else if (rightReady) {
		direction = left;
		switchStatus(approach, d.right);
	}
}

void LoopState::processEntering(int sensor, const Endpoint& from) {
	const LoopDef& d = def();
	Direction reversed = direction == left ? right : left;
	boolean movedToCentre = false;
	boolean movingReverse = false;

	if (from.hasSensor(sensor) && !from.isPrimedEnter()) {
		// moved from cross-edge to core only, or leaving back
		if (d.core.isPrimed()) {
			movedToCentre = true;
		}
	}
	if (d.core.hasSensor(sensor) && !d.core.isPrimed()) {
		if (from.occupied()) {
			movingReverse = true;
		}
	}
	if (movedToCentre) {
		if (debugTransitions) {
			Serial.println(F("Fully in core"));
		}
		switchStatus(moving, from);
	} else if (movingReverse) {
		if (debugTransitions) {
			Serial.println(F("Reversed"));
		}
		direction = reversed;
		switchStatus(exited, from);
	}
}

void LoopState::processMoving(int sensor, const Endpoint& from) {
	const LoopDef& d = def();
	const Endpoint& opp = d.opposite(from);
	if (d.core.hasSensor(sensor) && !d.core.isPrimed()) {
		if (from.hasChanged() && from.occupied()) {
			if (debugTransitions) {
				Serial.println(F("Guess: jumped back"));
			}
			direction = reversed();
			switchStatus(exited, from);
			return;
		}
		if (opp.hasChanged() && opp.occupied()) {
			if (debugTransitions) {
				Serial.println(F("Guess: jumped forward"));
			}
			switchStatus(exited, from);
			return;
		}
		if (debugTransitions) {
			Serial.println(F("Disappeared ! Running timeout."));
		}
		timeout = millis();
		return;
	}
	if (opp.hasSensor(sensor) && opp.isPrimedExit()) {
		if (debugTransitions) {
			Serial.println(F("Can exit loop"));
		}
		switchStatus(armed, opp);
		return;
	}
	if (from.hasSensor(sensor) && from.occupied()) {
		if (debugTransitions) {
			Serial.println(F("Turned back"));
		}
		direction = reversed();
		switchStatus(armed, from);
		switchStatus(exiting, from);
		return;
	}
}

void LoopState::processArmed(int sensor, const Endpoint& to) {
	const LoopDef& d = def();
	if (d.core.hasSensor(sensor) && !d.core.isPrimed()) {

	}
	if (!to.hasSensor(sensor)) {
		return;
	}
	if (to.changedOccupied(sensor, true)) {
		switchStatus(exiting, to);
	} else if (to.changedOccupied(sensor, false)) {
		if (debugTransitions) {
			Serial.println(F("Exit became invalid"));
		}
		switchStatus(moving, to);
		return;
	}
}

void LoopState::processExiting(int sensor, const Endpoint& via) {
	const LoopDef& d = def();
	if (d.core.hasSensor(sensor) && !d.core.isPrimed()) {
		switchStatus(exited, via);
		return;
	}
	if (via.hasSensor(sensor) && via.changedOccupied(sensor, false)) {
		direction = reversed();
		switchStatus(moving, via);
		return;
	}
}

void LoopState::processExited(int sensor, const Endpoint& via) {
	const LoopDef& d = def();
	if (d.core.hasSensor(sensor) && d.core.isPrimed()) {
		direction = reversed();
		switchStatus(entering, via);
		return;
	}
	if (via.hasSensor(sensor) && via.changedOccupied(sensor, false)) {
		switchStatus(idle, via);
		return;
	}
}

void LoopState::processChange(int sensor, boolean s) {
	const LoopDef& d = def();

	if (!d.active) {
		return;
	}
	if (d.occupiedTrackSensors() == 0) {
		// remain silent, maybe track power outage...
		switch (status) {
		case approach:
		case exited:
			break;
		default:
			return;
		}
	}
	if (debugTransitions) {
		Serial.print(F("Processing: ")); Serial.print(id() + 1);
		Serial.print(F(" Changed sensor: ")); Serial.println(sensor);
	}
	switch (status) {
		case idle:
			processIdle(sensor);
			break;
		case approach:
			processApproach(sensor, fromEdge());
			break;

		case entering:
			processEntering(sensor, fromEdge());
			break;

		case moving:
			processMoving(sensor, fromEdge());
			break;

		case armed:
			processArmed(sensor, toEdge());
			break;

		case exiting:
			processExiting(sensor, toEdge());
			break;

		case exited:
			processExited(sensor, toEdge());
			break;
	}
}

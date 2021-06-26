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

long outageTimeout = 5 * 60 * 1000l; // 5 minutes in the core
long outageAproachExitTimeout = 10 * 1000; // 10 seconds at the edges

boolean logTransitions = true;

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
 * Relay:	 OFF?		  ON					    		 ON                  OFF?       OFF
 * idle -> approach => readyEnter -> entering -> moving =>  armed -> exiting -> exited   -> idle
 * 									  |          |          \----\   |           |          |
 * 									  v			 v                \  v           v          v
 *			 						exited  <- exiting <- armed <- moving  <- entering <- approach <- idle
 *
 * And "Moving" and "Idle" are auto-transitional states:
 * - if exit conditions are met, Moving transitions to Armed
 * - if other endpoint's enter conditions are met, Idle transitions to Approach.
 *
 */

void LoopState::maybeArm(const Endpoint& via) {
	const LoopDef& d = def();
	if (!d.core.occupied()) {
		switchStatus(idle, via);
		return;
	}
	if (!via.isPrimedExit()) {
		if (debugTransitions) {
			Serial.println(F("Exit not ready"));
		}
		return;
	}
	const Endpoint& from = d.opposite(via);
	if (from.hasTriggerSensors()) {
		if (from.sensorsActive()) {
			if (debugTransitions) {
				Serial.println(F("In sensors still active"));
			}
			markDirSensor(false);
			return;
		} else if (!dirSensorTimeout(true)) {
			return;
		}
	}
	if (via.sensorOut > 0) {
		if (!via.sensorsActive()) {
			if (debugTransitions) {
				Serial.println(F("Exit out sensor NOT active"));
			}
			return;
		}
	}
	if (d.core.isDirectionPrimed(directionTo(via) == left)) {
		switchStatus(armed, via);
	}
}

void LoopState::processReadyEnter(int sensor, const Endpoint& ep) {
	processApproach(sensor, ep);
}

void LoopState::processApproach(int sensor, const Endpoint& from) {
	const LoopDef& d = def();
	if (from.hasSensor(sensor) && from.changedOccupied(sensor, false)) {
		if (d.core.isPrimed()) {
			if (debugTransitions) {
				Serial.println(F("Core section jumped to"));
			}
			switchStatus(moving, from);
			return;
		} else {
			if (debugTransitions) {
				Serial.println(F("Left & core is empty"));
			}
			switchStatus(idle, from);
		}
	}
	if (d.core.hasSensor(sensor)) {
		if (debugTransitions) {
			Serial.println(F("Core sensor changed"));
			Serial.println(direction == left ? F(" -> Left") : F("-> Right"));
			Serial.println(&from == &d.left ? F("Left") : F("Right"));
		}
		if (d.core.isPrimed()) {
			if (from.occupied()) {
				if (debugTransitions) {
					Serial.println(F("Core section partially entered"));
				}
				switchStatus(entering, from);
				return;
			} else {
				if (debugTransitions) {
					Serial.println(F("Jumped into core"));
				}
				switchStatus(moving, from);
				return;
			}
		}
	} else if (status == approach) {
		maybeReadyEnter(from);
	}
}

void LoopState::maybeApproach(Status prevStatus, const Endpoint& from) {
	if (!from.isPrimedEnter()) {
		return;
	}
	direction = directionFrom(from);
	switchStatus(approach, from);
}

void LoopState::maybeReadyEnter(const Endpoint& from) {
	boolean transition = false;

	if (from.isPrimedEnter()) {
		switchStatus(readyEnter, from);
	}
}

void LoopState::maybeFreeRelay(const Endpoint& via) {
	if (via.sensorIn > 0) {
		const Endpoint &opp = def().opposite(via);
		if ((opp.sensorIn == 0) && (opp.relay > 0)) {
			if (logTransitions) {
				Serial.print(F("Opposite has no sensor, conservative switch to opposite"));
			}
			switchRelayTo(opp);
		}
	}
}

void LoopState::switchStatus(Status s, const Endpoint& ep) {
	Status oldStatus = status;
	Direction oldDirection = direction;

	status = s;
	if (logTransitions) {
		Serial.print('#'); Serial.print(id() + 1);
		Serial.print(F(": Change status: ")); Serial.print(statName(oldStatus)); Serial.print(F(" => ")); Serial.print(statName(s));
		Serial.print(F(", Direction: ")); Serial.println(direction ? F("left") : F("right"));
	}
	const LoopDef& d = def();

	switch (s) {
		default:
			Serial.print(F("*Unhandled state: ")); Serial.print(s); Serial.print('-'); Serial.println(statName(s));
			break;

		case exited:
			maybeFreeRelay(ep);
			break;

		case approach:
			/*
			switchRelay(fromEdge().relay, fromEdge().relayTriggerState);
			markDirSensor(false);
			*/
			maybeReadyEnter(fromEdge());
			break;

		case readyEnter:
			switchRelay(fromEdge().relay, fromEdge().relayTriggerState);
			markDirSensor(false);
			break;

		case armed:
			direction = directionTo(ep);
			markDirSensor(true);
			switchRelay(toEdge().relay, toEdge().relayTriggerState);
			break;

		case idle:
			switchRelay(d.left.relay, d.left.relayOffState);
			switchRelay(d.right.relay, d.right.relayOffState);

			maybeApproach(oldStatus, d.opposite(ep));
			break;

		case moving:
			leftSensorTime = rightSensorTime = 0;
			direction = directionFrom(ep);
			if (oldStatus == entering) {
				markDirSensor(false);
			}
			const Endpoint& opp = d.opposite(ep);
			maybeArm(opp);
			break;
	}
	if (status == idle) {
		if (logTransitions) {
			Serial.println(F("Clearing trigger sensors"));
		}
		outageStart = 0;
		leftSensorTime = rightSensorTime = 0;
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
				Serial.print(F("Cold boot: core + left"));
			}
			switchStatus(armed, d.left);
			switchStatus(exiting, d.left);
		} else if (rightWasPrimed) {
			direction = right;
			if (debugTransitions) {
				Serial.print(F("Cold boot: core + right"));
			}
			switchStatus(armed, d.right);
			switchStatus(exiting, d.right);
		}
		return;
	}

	if (debugTransitions) {
		Serial.println(F("Checking left"));
	}
	boolean leftReady = d.left.hasSensor(sensor) && d.left.isValidEnter();
	if (debugTransitions) {
		Serial.println(F("Checking right"));
	}
	boolean rightReady = d.right.hasSensor(sensor) && d.right.isValidEnter();

	if (debugTransitions) {
		Serial.print(F("Left:  ")); d.left.printState();
		Serial.print(F("Right: ")); d.right.printState();
	}

	if (leftReady && rightReady) {
		// try to determine if one of the 'ready' endpoints is already primed:
		boolean leftPrimed = d.left.isPrimedEnter();
		boolean rightPrimed = d.right.isPrimedEnter();
		if (leftPrimed && !rightPrimed) {
			rightReady = false;
		} else if (rightPrimed && !leftPrimed) {
			leftReady = false;
		} else {
			// no op
			if (debugTransitions) {
				Serial.println(F("Both active => idle"));
			}
			return;
		}
	}
	if (leftReady) {
		direction = right;
		switchStatus(approach, d.left);
	} else if (rightReady) {
		direction = left;
		switchStatus(approach, d.right);
	}
}

boolean LoopState::dirSensorTimeout(boolean moveOut) const {
	const Endpoint ep = moveOut ? toEdge() : fromEdge();
	if (!ep.hasTriggerSensors()) {
		return true;
	}
	if (ep.sensorsActive()) {
		if (debugTransitions) {
			Serial.println(F("Sensors still active"));
		}
		return false;
	}
	long dst = dirSensorTime(moveOut);
	if (dst == 0) {
		return true;
	}
	long d = millis() - dst;
	if (debugTransitions) {
		Serial.print(F("Sensor diff: ")); Serial.println(d);
	}
	return d > def().sensorTimeout;
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
		switchStatus(exited, d.opposite(from));
	}
}

void LoopState::processMoving(int sensor, const Endpoint& from) {
	const LoopDef& d = def();
	const Endpoint& opp = d.opposite(from);

	if (maybeCoreAbandoned(sensor)) {
		return;
	}

	if ((opp.hasSensor(sensor) || from.hasTrigger(sensor))) {
		if (opp.isPrimedExit()) {
			if (debugTransitions) {
				Serial.println(F("Can exit loop"));
			}
			if (from.hasTriggerSensors() && !opp.hasTriggerSensors()) {
				if (debugTransitions) {
					Serial.println(F("Checking past sensors"));
				}
				if (!dirSensorTimeout(false)) {
					if (debugTransitions) {
						Serial.println(F("* Still In Timeout"));
					}
					return;
				}
			}
			switchStatus(armed, opp);
			return;
		}
	}
	if (from.hasSensor(sensor)) {
		if (from.isPrimedExit()) {
			direction = reversed();
			switchStatus(armed, from);
		} else if (from.occupied()) {
			if (debugTransitions) {
				Serial.println(F("Turned back"));
			}
			direction = reversed();
			switchStatus(armed, from);
			switchStatus(exiting, from);
			return;
		}
	}
}

boolean LoopState::maybeCoreAbandoned(int sensor) {
	const LoopDef& d = def();
	if (d.core.isPrimed()) {
		return false;
	}
	if (d.left.occupied()) {
		if (!d.right.occupied()) {
			direction = left;
			if (debugTransitions) {
				Serial.println(F("Core abandoned going left"));
			}
			switchStatus(exited, d.left);
			return true;
		}
	} else if (d.right.occupied()) {
		direction = left;
		if (debugTransitions) {
			Serial.println(F("Core abandoned going right"));
		}
		switchStatus(exited, d.right);
		return true;
	}
	if (debugTransitions) {
		Serial.println(F("Disappeared ! Running timeout."));
	}
	timeout = millis();
	return true;
}

void LoopState::switchRelayTo(const Endpoint& exitVia) {
	if (exitVia.relay > 0) {
		switchRelay(exitVia.relay, exitVia.relayTriggerState);
		return;
	}
	const Endpoint& opp = def().opposite(exitVia);
	if (opp.relay > 0) {
		switchRelay(opp.relay, !opp.relayTriggerState);
	}
}

void LoopState::processArmed(int sensor, const Endpoint& to) {
	const LoopDef& d = def();
	const Endpoint& from = d.opposite(to);

	if (maybeCoreAbandoned(sensor)) {
		return;
	}
	boolean revert = false;
	if (!to.hasSensor(sensor)) {
		if (from.sensorOut == sensor) {
			if (logTransitions) {
				Serial.print(F("Train reversed while armed for ")); Serial.println(direction == left ? "left" : "right");
			}
			revert = true;
		}
		if (!revert) {
			return;
		}
	}
	if (to.changedOccupied(sensor, true)) {
		switchStatus(exiting, to);
		return;
	}
	if (to.isValidExit()) {
		if (to.hasTriggerSensors() && dirSensorTimeout(true)) {
			if (debugTransitions) {
				Serial.println(F("Trigger sensor timeout"));
			}
			revert = true;
		}
	}
	if (!to.isPrimedExit()) {
		if (debugTransitions) {
			Serial.println(F("Exit became invalid"));
		}
		revert = true;
	}

	if (revert) {
		if (to.sensorOut > 0) {
			// exit has safety sensor, no need to change direction
			if (from.sensorOut == 0) {
				// but the opposite has no sensor; flip the relay just in case.
				switchRelayTo(from);
				if (debugTransitions) {
					Serial.println(F("Relay switched to opposite"));
				}
			}
			switchStatus(moving, to);
			return;
		}

		// still moving in the _SAME_ direction; exit became invalid for
		// some reason. Heading to the exit, but just in case turn off the relay.
		switchStatus(moving, to);
	}
}

void LoopState::processExiting(int sensor, const Endpoint& via) {
	const LoopDef& d = def();
	if (!d.core.isPrimed()) {
		if (via.hasTriggerSensors()) {
			if (debugTransitions) {
				Serial.println(F("Exit has trigger sensors"));
			}
			if (via.sensorsActive()) {
				if (debugTransitions) {
					Serial.println(F("* Out sensor still active"));
				}
				markDirSensor(true);
				return;
			} else if (!dirSensorTimeout(true)) {
				if (debugTransitions) {
					Serial.println(F("* In Out timeout"));
				}
				return;
			}
		}
		switchStatus(exited, via);
		return;
	}
	if (via.hasSensor(sensor) && via.changedOccupied(sensor, false)) {
		// moving takes "from" as parameter, so 'via' will revers direction
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
	if (via.sensorIn == sensor) {
		if (readS88(via.sensorIn) != via.invertInSensor) {
			direction = directionFrom(via);
			switchStatus(approach, via);
			return;
		}
	}
}

void LoopState::processOccupied(int sensor, boolean s) {
	const LoopDef& d = def();

	if (d.left.hasSensor(sensor) && d.left.isPrimedExit()) {
		direction = left;
		if (d.left.occupiedTrackSensors() == 0) {
			switchStatus(moving, /* from */ d.right);
		} else {
			switchStatus(exiting, d.left);
		}
		return;
	}
	if (d.right.hasSensor(sensor) && d.right.isPrimedExit()) {
		direction = right;
		if (d.right.occupiedTrackSensors() == 0) {
			switchStatus(moving, /* from */ d.left);
		} else {
			switchStatus(exiting, d.right);
		}
		return;
	}

	if (d.occupiedTrackSensors() == 0) {
		switchStatus(idle, d.left);
		return;
	}
	if (d.core.occupied()) {
		return;
	}
	if (d.left.hasSensor(sensor) && d.left.occupied()) {
		direction = left;
		switchStatus(exited, d.left);
		return;
	}
	if (d.right.hasSensor(sensor) && d.right.occupied()) {
		direction = right;
		switchStatus(exited, d.right);
		return;
	}
}

void LoopState::handleOutage() {
	if (!outage()) {
		return;
	}
	long delta = millis() - outageStart;
	long threshold;

	if ((status == approach || status == exited)) {
		threshold = outageAproachExitTimeout;
	} else if (status == readyEnter && (!fromEdge().hasTriggerSensors() || !fromEdge().isPrimedEnter())) {
		threshold = outageAproachExitTimeout;
	} else {
		threshold = outageTimeout;
	}

	const LoopDef& d = def();
	if (delta > threshold) {
		Serial.print('#'); Serial.print(id() + 1);
		Serial.print(F(" delta = ")); Serial.print(delta); Serial.print(F(", threshold = ")); Serial.print(threshold);
		Serial.println(F(": Outage timeout => idle"));
		switchStatus(idle, d.left);
	}
}

void LoopState::processChange(int sensor, boolean s) {
	const LoopDef& d = def();

	if (!d.active) {
		return;
	}
	boolean active = d.occupiedTrackSensors();
	if (active) {
		if (outage() && status != idle) {
			if (!d.core.occupied()) {
				const Endpoint& ep = toEdge();
				if (ep.occupiedTrackSensors() == 0) {
					Serial.println(F("Outage recovery => idle"));
					switchStatus(idle, d.left);
					return;
				}
			}
			Serial.print(F("Outage ended => ")); Serial.print(statName(status)); Serial.println();
		}
	} else {
		// remain silent, maybe track power outage...
		switch (status) {
		/*
		case approach:
		*/
		case exited:
		case idle:
			break;
		default:
			if (outageStart == 0) {
				Serial.print('#'); Serial.print(id() + 1);
				Serial.println(F(": Outage start"));
				outageStart = millis();
				return;
			}
			handleOutage();
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

		case occupied:
			processOccupied(sensor, s);
			break;

		case approach:
			processApproach(sensor, fromEdge());
			break;

		case readyEnter:
			processReadyEnter(sensor, fromEdge());
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

	// keep sensor timeouts:
	switch (status) {
		case approach: case entering:
			{
				const Endpoint& fe = fromEdge();
				if (fe.hasTriggerSensors() && fe.sensorsActive()) {
					markDirSensor(false);
				}
			}
			break;

		case moving:
			{
				const Endpoint& fe = fromEdge();
				if (fe.hasTriggerSensors() && fe.sensorsActive()) {
					markDirSensor(false);
				}
			}

			// FALL THROUGH

		case armed: case exiting: case exited:
			const Endpoint& te = toEdge();
			if (te.hasTriggerSensors() && te.sensorsActive()) {
				markDirSensor(true);
			}
			break;

	}

	if (active) {
		outageStart = 0;
	}
}

/*
 * Loop.h
 *
 *  Created on: Mar 11, 2021
 *      Author: sdedic
 */

#ifndef LOOPS_H_
#define LOOPS_H_

#include <Arduino.h>

const int maxLoopCount = 8;
const int maxRelayCount = 4;

typedef int (*sensorIteratorFunc)(int sensorId);
extern int freeUnusedSensors();
extern int relayPins[maxRelayCount];

struct LoopState;
struct LoopDef;
extern LoopDef loopDefinitions[];
extern LoopState loopStates[];

/**
 * Each Endpoint may be made up from:
 * - two detected track segments: they join by a turnout that is a part of the switched loop.
 * - a segment, and a switch, that switches to the loop
 * - a track segment (or two) and a sensor that is triggered just before the loop's entry
 */
struct Endpoint {
	/**
	 * The approaching track, or one section of two possible
	 * approaching tracks, joined (and selected) by 'turnout'.
	 */
	byte sensorA: 8;

	/**
	 * If != 0, the other alternative approaching track, selected by 'turnout'.
	 */
	byte sensorB: 8;

	/**
	 * Turnout feedback "sensor"; if nont 0, selects 'sensorA' or 'sensorB' to be read as
	 * the approaching track section.
	 */
	byte turnout : 8;

	/**
	 * Sensor that triggers on inbound move. 9 = not defined.
	 */
	byte sensorIn : 8;

	/**
	 * Sensor that triggers on outbound move. 9 = not defined.
	 */
	byte sensorOut : 8;

	/**
	 * Additional joined track.
	 */
	byte shortTrack : 8;

	int switchOrSensor : 8;
	boolean useSwitch : 1;
	boolean invertSensor : 1;
	boolean invertA : 1;
	boolean invertB : 1;
	boolean invertShortTrack: 1;
	boolean invertInSensor : 1;
	boolean invertOutSensor : 1;
	boolean invertTurnout : 1;
	boolean triggerState : 1;

	/**
	 * Checks if the endpoint is 'primed' for exit. The adjacent track must not be occupied, the turnout must
	 * be in correct position and trigger sensor must signal.
	 */
	boolean isPrimedExit() const;

	boolean isValidEnter() const;

	boolean isValidExit() const;

	/**
	 * Checks if the endpoint is 'primed' for loop enter.
	 * Track must be occupied, the turnout (if defined) in a correct position, and trigger sensor (if defined)
	 * must signal.
	 */
	boolean isPrimedEnter() const;

	boolean changedOccupied(int sensor, boolean occupied) const;

	boolean occupied() const;

	int selectedExitTrack() const;

	boolean hasChanged() const;

	boolean stateA() const;

	boolean stateB() const;

	boolean hasSensor(int id) const {
		return (sensorA == id) || (sensorB == id) || (switchOrSensor == id) ||
			   (turnout == id) || (sensorIn == id) || (sensorOut == id) || (shortTrack == id);
	}

	void printState() const;

	int forSensors(sensorIteratorFunc fn) const;

	int occupiedTrackSensors() const;

	Endpoint() :sensorA(0), invertA(false),
				sensorB(0), invertB(false),
				switchOrSensor(0), invertSensor(false), useSwitch(false),
				turnout(0), invertTurnout(false),
				sensorIn(0), invertInSensor(false),
				sensorOut(0), invertOutSensor(false),
				shortTrack(0), invertShortTrack(false),

				triggerState(false) {}
};

struct LoopCore {
	int track : 8;
	int sensorA : 8;
	int sensorB : 8;

	boolean invertTrack : 1;
	boolean invertA  : 1;
	boolean invertB  : 1;

	boolean isPrimed() const;
	boolean hasChanged() const;
	boolean hasSensor(int id) const {
		return (track == id) || (sensorA == id) || (sensorB == id);
	}
	boolean occupied() const;
	void printState() const;
	int forSensors(sensorIteratorFunc fn) const;
	LoopCore() : track(0), sensorA(0), sensorB(0), invertA(false), invertB(false), invertTrack(false) {}

	int occupiedTrackSensors() const;
};

/**
 * State transitions:
 *
 * Continuous movement / with pauses:
 * - idle -> enteringLeft -> enteredLeft -> exitingRight -> idle
 * - idle -> enteringRight -> enteredRight -> exitingLeft-> idle
 *
 * - idle -> enteringLeft -> idle : The left endpoint activated, then deactivated without train entering core.
 * 									Power outage, or the train turned and left the endpoint, no action / reset to default.
 * - idle -> enteringRight -> idle : similar for right.
 *
 * - idle -> enteringLeft -> enteredLeft -> idle: the train moved  inside the core, but then turned and exited back left.
 *
 */

enum Direction {
	right = 0,
	left = 1
};

enum Status {
	idle = 0,

	/**
	 * Entering the loop from the right side. In the edge track
	 */
	approach,

	/**
	 * Has entered the loop from the right side, moving to the left. Partially in the edge & core tracks
	 */
	entering,

	/**
	 * Fully in the core track
	 */
	moving,

	/**
	 * Exiting the loop to the left, fully in the core track
	 */
	armed,

	/**
	 * Exiting to the left, in core & edge tracks
	 */
	exiting,

	/**
	 * Exiting the loop to the left, fully in edge track
	 */
	exited,
	/**
	 * The loop is occupied with no idea if moving left or right.
	 */
	occupied
};

struct LoopDef {
	Endpoint left;
	Endpoint right;
	LoopCore	 core;

	boolean  active : 1;
	int		 relayA : 3;
	int	     relayB : 3;

	int id() const {
		return this - loopDefinitions;
	}

	void printState() const;

	int forSensors(sensorIteratorFunc fn) const;

	boolean hasSensor(int id) const {
		return left.hasSensor(id) || right.hasSensor(id) || core.hasSensor(id);
	}

	LoopDef() : active(false), relayA(0), relayB(0) {}

	const Endpoint& opposite(const Endpoint& ep) const { return &ep == &left ? right : left; }
	void defineSensors() const;
	static void printAllStates();

	int occupiedTrackSensors() const;
};

struct LoopState {
	int id() const {
		return this - loopStates;
	}

	Direction reversed() const {
		return direction == left ? right : left;
	}

	Direction directionTo(const Endpoint& ep) {
		return &(def().left) == &ep ? left : right;
	}

	Direction directionFrom(const Endpoint& ep) {
		return &(def().left) == &ep ? right : left;
	}

	const LoopDef& def() const {
		int i = id();
		if (i >= maxLoopCount) {
			i = 0;
		}
		return loopDefinitions[i];
	}

	const Endpoint& fromEdge() const {
		if (direction == left) {
			return def().right;
		} else {
			return def().left;
		}
	}

	const Endpoint& toEdge() const {
		if (direction == left) {
			return def().left;
		} else {
			return def().right;
		}
	}

	Status status : 4;
	Direction direction : 1;
	long timeout;
	long outageStart;

	LoopState() : status(Status::idle), direction(left), timeout(0), outageStart(0) {}

	boolean outage() const { return outageStart > 0; };

	void switchStatus(Status s, const Endpoint& e);
	void processChange(int sensor, boolean state);
	void printState() const;

	void processIdle(int sensor);
	void processApproach(int sensor, const Endpoint& ep);
	void processEntering(int sensor, const Endpoint& from);
	void processMoving(int sensor, const Endpoint& from);
	void processArmed(int sensor, const Endpoint& to);
	void processExiting(int sensor, const Endpoint& to);
	void processExited(int sensor, const Endpoint& to);

	void maybeArm(const Endpoint& via);
	void maybeApproach(Status prevStatus, const Endpoint& from);
};

boolean defineLoop(int id, const LoopDef& def);

String statName(Status s);
String statName(Status s, Direction d);
void switchRelay(int rid, boolean on);
boolean isRelayOn(int rid);

#endif /* LOOPS_H_ */

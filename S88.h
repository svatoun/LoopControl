/*
 * S88.h
 *
 *  Created on: Mar 11, 2021
 *      Author: sdedic
 */

#ifndef S88_H_
#define S88_H_

const int LOAD_INT_0      = 2 ;        // 2 LOAD 0 int
const int CLOCK_INT_1     = 3 ;        // 3 CLOCK 1 int
const int DATA_IN         = 4 ;        // data in
const int DATA_OUT        = 5 ;        // data out

const int s88MaxSize	  = 32;		   // max number of 8bit S88 modules

const int maxSensorCount  = 8 * 3;	   // maximum number of sensors

/**
 * Milliseconds the sensor's state must hold in order to report a change.
 */
extern int sensorDebounceMillis;

/**
 * Callback to be called if the sensor's state changes.
 */
typedef void (*sensorChangeFunc)(int sensor, boolean state);
typedef int (*sensorIteratorFunc)(int sensorId);


extern sensorChangeFunc sensorCallback;

void s88InLoop();
void s88LoadInt();
void s88ClockInt();
boolean readS88(int sensor);
int tryReadS88(int sensor);
boolean defineSensor(int sensor);
boolean freeSensor(int sensor);
int forSensors(sensorIteratorFunc fn);
boolean s88Changed(int sensor);
void overrideS88(int sensorId, boolean override, boolean state);

struct Sensor {
	/**
	 * State reported from reading
	 */
	boolean reportState : 1;

	/**
	 * State reported by S88, not necessarily stable
	 */
	boolean s88State : 1;

	/**
	 * Should trigger a change notification
	 */
	boolean	triggerChange : 1;

	/**
	 *change is being processed
	 */
	boolean	changeProcessing : 1;

	/**
	 *
	 */
	boolean overriden : 1;

	/**
	 * True, if the sensor is transitioning
	 */
	boolean changing : 1;

	/**
	 * Lowest 32 bits from millis of the last change of S88 state.
	 */
	unsigned int	stableFrom = 0;

	/**
	 * The sensor number
	 */
	byte sensorId = 0;

	Sensor() : reportState(false), s88State(false), triggerChange(false), changing(false), changeProcessing(false), overriden(false) {}
	Sensor(int id) : sensorId(id), reportState(false), s88State(false), triggerChange(false), changing(false), changeProcessing(false), overriden(false) {}

	boolean isDefined() const { return sensorId != 0; }
	static void printAll(boolean includeNone);
	void print() const;
	void clear() { sensorId = 0; }
};

extern Sensor sensors[];

#endif /* S88_H_ */

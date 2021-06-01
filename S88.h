/*
 * S88.h
 *
 *  Created on: Mar 11, 2021
 *      Author: sdedic
 */

#ifndef S88_H_
#define S88_H_

#include "PinOut.h"

const int s88MaxSize	  = 32;		   // max number of 8bit S88 modules

const int s88MaxSize_bytes = s88MaxSize;


const int maxSensorCount  = 8 * 3;	   // maximum number of sensors
const int maxSensorId = s88MaxSize * 8;

/**
 * Milliseconds the sensor's state must hold in order to report a change.
 */
extern int sensorDebounceMillis;

/**
 * Callback to be called if the sensor's state changes.
 */
typedef void (*sensorChangeFunc)(int sensor, boolean state);
typedef int (*sensorIteratorFunc)(int sensorId, boolean triggerType);


extern sensorChangeFunc sensorCallback;

void s88InLoop();
void s88LoadInt();
void s88ClockInt();
boolean readS88(int sensor);
int tryReadS88(int sensor);
boolean defineSensor(int sensor);
boolean defineSensor(int id, boolean triggerType);
boolean freeSensor(int sensor);
int forSensors(sensorIteratorFunc fn);
boolean s88Changed(int sensor);
void overrideS88(int sensorId, boolean override, boolean state);

struct SensorTiming {
	int trackUpDebounce = 1;
	int trackDownDebounce = 800;

	int triggerUpDebounce = 5;
	int triggerDownDebounce = 200;
};

extern SensorTiming defaultTiming;

struct Sensor;

struct SensorData {
	unsigned int	sensorUpDebounce;

	unsigned int	sensorDownDebounce;
	/**
	 * The sensor number
	 */
	byte sensorId;

	boolean triggerSensor;

	SensorData(const Sensor& from);
};

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
	volatile boolean	triggerChange : 1;

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

	boolean triggerSensor : 1;

	/**
	 * Lowest 32 bits from millis of the last change of S88 state.
	 */
	unsigned int	stableFrom = 0;

	unsigned int	sensorUpDebounce = 0;

	unsigned int	sensorDownDebounce = 0;
	/**
	 * The sensor number
	 */
	byte sensorId = 0;

	Sensor() : reportState(false), s88State(false), triggerChange(false), changing(false), changeProcessing(false), overriden(false), triggerSensor(false) {}
	Sensor(int id) : sensorId(id), reportState(false), s88State(false), triggerChange(false), changing(false), changeProcessing(false), overriden(false), triggerSensor(false) {}
	Sensor(const SensorData& data);

	SensorData data() { return SensorData(*this); }

	boolean isDefined() const { return sensorId != 0; }
	static void printAll(boolean includeNone);
	void print() const;
	void clear() { sensorId = 0; }

	void dumpTimeouts() const;

	int upDebounceTime() const;
	int downDebounceTime() const;
};

extern Sensor sensors[];

#endif /* S88_H_ */

/*
 * Turrent.h
 *
 *  Created on: Apr 23, 2019
 *      Author: developer
 */

#ifndef TURRENT_H_
#define TURRENT_H_

#include "StudentsRobot.h"
#include <Stepper.h>

#define FIRE_DURATION_MS 10000 // how long to fire in milliseconds
#define FIRE_SPEED 400 //the speed in degrees per second to rotate the motor

#define SWEEP_STEP_RANGE 10 //the range of steps to sweep

enum TurrentState {
	TURN_TURRENT, FIRE, SWEEP, STANDBY_TURRENT
};

class Turrent {
private:
	PIDMotor *firingMotor; // motor for firing
	Stepper *stepper; //motor for turning

	TurrentState state;
	long fireStartTime;

	int currentSweepSteps;
	bool sweepRight;

	void sweeping();
public:
	Turrent(PIDMotor*, Stepper *stepper);
	virtual ~Turrent();

	/**
	 * This function should ping the IR camera
	 * @return true if fire detected, false if fire is not detected
	 */
	bool checkFire();

	/**
	 * This function should fire the air cannon.
	 */
	void fire();


	/**
	 * Perform the sweeping motion at the current position of the turrent
	 */
	void sweep();

	/**
	 * Called in StudentRobot should be used to update stepper and time firings
	 */
	void loop();

};

#endif /* TURRENT_H_ */

/*
 * Turrent.cpp
 *
 *  Created on: Apr 23, 2019
 *      Author: developer
 */

#include "Turrent.h"

Turrent::Turrent(PIDMotor *firingMotor, Stepper *stepper) {
	this->firingMotor = firingMotor;
	this->stepper = stepper;

	this->state = TurrentState::STANDBY_TURRENT;

	this->fireStartTime = 0;
}

Turrent::~Turrent() {}

void Turrent::loop() {
	if(state == TurrentState::STANDBY_TURRENT) {
		return;
	}

	if(state == TurrentState::FIRE) {
		if(millis() - this->fireStartTime > FIRE_DURATION_MS) {
			this->firingMotor->stop();
			this->state = TurrentState::STANDBY_TURRENT;
		}
	}

	if(state == TurrentState::TURN_TURRENT) {
		//Not sure in necessary but i'll keep it here for now. - LT (4/23/2019)
	}

	if(state == TurrentState::SWEEP) {
		sweeping();
	}
}

void Turrent::fire() {
	this->state = FIRE;
	this->fireStartTime = millis();
	this->firingMotor->setVelocityDegreesPerSecond(FIRE_SPEED);
}

bool Turrent::checkFire() {
	return false;
}

void Turrent::sweep() {
	this->currentSweepSteps = 0;
	this->state = TurrentState::SWEEP;
	this->sweepRight = true;
}

void Turrent::sweeping() {
	if(this->sweepRight) {
		if(this->currentSweepSteps < SWEEP_STEP_RANGE) {
			stepper->step(1); // move the stepper motor to the next position
		} else {
			//move the stepper motor SWEEP_STEP_RANGE steps to origin point
		}
	}

}


/*
 * Turrent.cpp
 *
 *  Created on: Apr 23, 2019
 *      Author: developer
 */

#include "Turrent.h"

Turrent::Turrent(PIDMotor *firingMotor, BasicStepperDriver *stepper) {
	this->firingMotor = firingMotor;
	this->stepper = stepper;

	this->stepper->begin(1, 1);

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

	if(state == TurrentState::SWEEP) {
		sweeping();
	}

	if(state == TurrentState::TURN_TURRENT) {
		if(this->stepper->getStepsRemaining() == 0) {
			this->state = TurrentState::STANDBY_TURRENT;
		}
	}
}

void Turrent::fire() {
	this->state = FIRE;
	this->fireStartTime = millis();
	this->firingMotor->setVelocityDegreesPerSecond(FIRE_SPEED);
}

bool Turrent::checkFire(){
	return false;
}

void Turrent::sweep() {
	this->currentSweepSteps = 0;
	this->state = TurrentState::SWEEP;
	this->sweepRight = true;

	this->stepper->startRotate(90);
}

void Turrent::sweeping() {
	if(this->sweepRight) {
		if(this->stepper->getStepsRemaining() == 0) {
			checkFire();

			this->sweepRight = false;
			this->stepper->startRotate(-180);
		}
	}

	if(this->sweepRight == false) {
		if(this->stepper->getStepsRemaining() == 0) {
			checkFire();
			this->stepper->startRotate(90);

			this->state = TurrentState::TURN_TURRENT;
		}
	}

}


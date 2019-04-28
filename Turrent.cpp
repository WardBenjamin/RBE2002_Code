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
}

void Turrent::fire() {
	this->state = FIRE;
	this->fireStartTime = millis();
}

bool Turrent::checkFire(){
	return false;
}

void Turrent::sweep() {
	this->state = TurrentState::SWEEP;

	this->stepper->rotate(90);

	if(checkFire()) {
		fire();
		return;
	}

	this->stepper->rotate(-90);
	this->stepper->rotate(-90);

	if(checkFire()) {
		fire();
		return;
	}

	this->stepper->rotate(90); // reset to the starting positon

	this->state = TurrentState::STANDBY_TURRENT;

}



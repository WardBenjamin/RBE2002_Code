/*
 * Turrent.cpp
 *
 *  Created on: Apr 23, 2019
 *      Author: developer
 */

#include "Turrent.h"

Turrent::Turrent(PIDMotor *firingMotor, BasicStepperDriver *stepper, IRCamSimplePacketComsServer * IRCamera) {
	this->firingMotor = firingMotor;
	this->stepper = stepper;

	this->stepper->begin(10, 1);

	this->state = TurrentState::STANDBY_TURRENT;

	this->fireStartTime = 0;

	this->IRCamera = IRCamera;
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



	for(int x = 0; x < 4; x++) {
		if(IRCamera->readY(x) < 900) {
			Serial.println("[Turrent] The FIRE HAS BEEN FOUND");
			fire();
			return true;
		}
 	}

	return false;
}

void Turrent::sweep() {
	this->state = TurrentState::SWEEP;

//	this->stepper->rotate(100);

	if(checkFire()) {
		fire();
		return;
	}

	//this->stepper->rotate(-100);
	//this->stepper->rotate(-100);

	if(checkFire()) {
		fire();
		return;
	}


	this->state = TurrentState::STANDBY_TURRENT;

}



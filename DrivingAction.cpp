/*
 * DrivingAction.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#include "DrivingAction.h"

DrivingAction::DrivingAction(Action action, float param) {
	this->action = action;
	this->param = param;

	this->next = nullptr;
}

DrivingAction::~DrivingAction() {
	// TODO Auto-generated destructor stub
}

void DrivingAction::setNextDrivingAction(DrivingAction *next) {
	this->next = next;
}

DrivingAction* DrivingAction::getNextDrivingAction() { return *next; }

void DrivingAction::perform(DrivingChassis *chassis) {
	if(action == TURN) {
		chassis->turnDegrees(param, param * MS_PER_DEGREE);
	} else if(action == DRIVE) {
		chassis->driveForward(param, param * MS_PER_MILIMETER);
	}
}


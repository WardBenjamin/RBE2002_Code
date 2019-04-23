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

DrivingAction* DrivingAction::getNextDrivingAction() {
	return next;
}

void DrivingAction::perform(DrivingChassis *chassis) {
	Serial.println("Performing Action: \n\t");
	printDrivingAction();

	if (action == TURN) {
		float currentAngle = chassis->getIMU()->getAngle();
		float degreesToTurn = param - currentAngle;

		if(degreesToTurn < 0) {
			Serial.println("Robot should turn left.");
		} else if(degreesToTurn > 0) {
			Serial.println("Robot should turn right.");
		} else {
			Serial.println("Robot is already at the angle and thus shouldn't move");
		}

		chassis->turnDegrees(degreesToTurn, degreesToTurn * MS_PER_DEGREE);

	} else if (action == DRIVE) {
		chassis->driveForward(param, param * MS_PER_MILIMETER);
	}
}
void DrivingAction::printDrivingAction() {
	if (this->action == Action::TURN) {
		Serial.print(
				"TURN to " + String(getValue()) + " degrees\n");
	} else {
		Serial.print(
				"DRIVE for " + String(getValue()) + " mm\n");
	}
}


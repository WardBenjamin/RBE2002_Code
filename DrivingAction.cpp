/*
 * DrivingAction.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#include "DrivingAction.h"

DrivingAction::DrivingAction(Action action, float param, Node *end) {
	this->action = action;
	this->param = param;
	this->next = nullptr;

	this->endNode = end;
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

void DrivingAction::perform(DrivingChassis *chassis, Turrent *turrent) {
	Serial.println("Performing Action: \n\t");
	printDrivingAction();

	if (action == TURN) {

		float currentAngle = chassis->getIMU()->getAngle();
		Serial.println(currentAngle);

		float degreesToTurn = param - currentAngle;
		chassis->turnDegrees(degreesToTurn, abs(degreesToTurn) * MS_PER_DEGREE);

	} else if (action == Action::DRIVE) {
		chassis->driveForward(param, param * MS_PER_MILIMETER);
	} else if(action == Action::CHECK) {
		turrent->checkFire();
	} else if(action == Action::STOP) {
		chassis->driveForward(0, param);
	}
}
void DrivingAction::printDrivingAction() {
	if (this->action == Action::TURN) {
		Serial.print(
				"TURN to " + String(getValue()) + " degrees\n");
	} else if(action == Action::DRIVE) {
		Serial.print(
				"DRIVE for " + String(getValue()) + " mm\n");
	} else if(action == Action::CHECK) {
		Serial.print("CHECK\n");
	}
}


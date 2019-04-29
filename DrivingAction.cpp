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

DrivingAction::~DrivingAction() {}

void DrivingAction::setNextDrivingAction(DrivingAction *next) {
	this->next = next;
}

DrivingAction* DrivingAction::getNextDrivingAction() {
	return next;
}

void DrivingAction::perform(DrivingChassis *chassis, Turrent *turrent) {
	Serial.print("[DrivingAction] Performing action ");
	printDrivingAction();

	if (action == TURN) {

		float currentAngle = chassis->getIMU()->getAngle();
		float degreesToTurn = param - currentAngle;
		chassis->turnDegrees(degreesToTurn, abs(degreesToTurn) * MS_PER_DEGREE);

	} else if (action == Action::DRIVE) {

		if(param != 0) {
			chassis->driveForward(param, param * MS_PER_MILIMETER);
		} else {
			chassis->driveForward(param, 2000);
		}
	} else if(action == Action::CHECK) {
		turrent->sweep();
	}
}
void DrivingAction::printDrivingAction() {
	if (this->action == Action::TURN) {
		Serial.print(
				"TURN to " + String(getValue()) + " degrees @ Node #" + String(endNode->getID()) + "\n");
	} else if(action == Action::DRIVE) {
		Serial.print(
				"DRIVE for " + String(getValue()) + " mm @ Node #" + String(endNode->getID()) + "\n");
	} else if(action == Action::CHECK) {
		Serial.print("CHECK both sides for the fire @ Node #" + String(endNode->getID()) + "\n" );
	}
}


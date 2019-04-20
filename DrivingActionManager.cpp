/*
 * DrivingActionManager.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#include "DrivingActionManager.h"

DrivingActionManager::DrivingActionManager(DrivingChassis *chassis) {
	this->head = nullptr;
	this->chassis = chassis;
}

DrivingActionManager::~DrivingActionManager() {
	// TODO Auto-generated destructor stub
}

bool DrivingActionManager::hasNext() {
	return head != nullptr;
}
void DrivingActionManager::addDrivingAction(DrivingAction *action) {
	if(head == nullptr) {
		head = action;
		return;
	}

	DrivingAction *current = head;

	while(current->getNextDrivingAction() != nullptr) {
		current = current->getNextDrivingAction();
	}

	current->setNextDrivingAction(action);
}
void DrivingActionManager::performNextAction() {
	if(head != nullptr) {
		DrivingChassis *new_head = head->getNextDrivingAction();

		head->perform(this->chassis);

		head = new_head;
	}
}

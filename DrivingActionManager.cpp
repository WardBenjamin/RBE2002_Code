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

	this->graph = new Graph();
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
	if(head != nullptr && !isnan(this->chassis->getIMU()->getAngle()) && !isinf(this->chassis->getIMU()->getAngle())) {
		DrivingAction* new_head = head->getNextDrivingAction();

		if(head->getAction() == Action::DRIVE) {
			//check for the roadblock to make sure that it is safe
			float forwardDistance = this->chassis->getRangeFinder()->readSensorMM();

			/*if(forwardDistance < head->getValue() + 10) { //there is probably a road block
				//now reroute the route

				Node *badNode = head->getEndNode();

				badNode->deleteEdge(NORTH_INDEX);
				badNode->deleteEdge(SOUTH_INDEX);
				badNode->deleteEdge(WEST_INDEX);
				badNode->deleteEdge(EAST_INDEX);

			}*/
		}

		head->perform(this->chassis);
		head = new_head;
	}
}

void DrivingActionManager::loop() {
	if(chassis->getState() == DRIVING) {
		chassis->loop();
	} else {
		if(hasNext()) {
			performNextAction();
		}
		chassis->loop();
	}
}
void DrivingActionManager::pathToDrivingActions(Node *end) {
	std::vector<DrivingAction *> actions;

		Node *curr = end;

		curr->printNode();

		while (curr != nullptr && curr->predecessor != nullptr) {
			Node::Edge *edge = graph->findConnectingEdge(curr->predecessor, curr);

			actions.insert(actions.begin(),
					new DrivingAction(DRIVE, edge->lengthX + edge->lengthY, curr));

			if (curr->predecessor->getEastEdge() == edge) {
				actions.insert(actions.begin(), new DrivingAction(TURN, 90, curr->predecessor));
			} else if (curr->predecessor->getWestEdge() == edge) {
				actions.insert(actions.begin(), new DrivingAction(TURN, 270, curr->predecessor));
			} else if (curr->predecessor->getNorthEdge() == edge) {
				actions.insert(actions.begin(), new DrivingAction(TURN, 0, curr->predecessor));
			} else if (curr->predecessor->getSouthEdge() == edge) {
				actions.insert(actions.begin(), new DrivingAction(TURN, 180, curr->predecessor));
			}
			curr = curr->predecessor;
		}

		for (int x = 0; x < actions.size(); x++) {
			this->addDrivingAction(actions.at(x)); //add all of the items to the paths

			Serial.print(String(x) + ". ");
			actions.at(x)->printDrivingAction();
		}
}
void DrivingActionManager::setPath(int xStart, int yStart, int xEnd, int yEnd) {
	this->head = nullptr; //clear the previous path

	Node *end = this->graph->setBestPath(xStart, yStart, xEnd, yEnd);
	pathToDrivingActions(end);
}

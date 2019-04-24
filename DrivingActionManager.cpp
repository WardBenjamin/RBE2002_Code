/*
 * DrivingActionManager.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#include "DrivingActionManager.h"

DrivingActionManager::DrivingActionManager(DrivingChassis *chassis,
		Turrent *turrent) {
	this->head = nullptr;
	this->chassis = chassis;
	this->turrent = turrent;

	this->graph = new Graph();
	srand(time(0));
}

DrivingActionManager::~DrivingActionManager() {
	// TODO Auto-generated destructor stub
}

bool DrivingActionManager::hasNext() {
	return head != nullptr;
}
void DrivingActionManager::addDrivingAction(DrivingAction *action) {
	if (head == nullptr) {
		head = action;
		return;
	}

	DrivingAction *current = head;

	while (current->getNextDrivingAction() != nullptr) {
		current = current->getNextDrivingAction();
	}

	current->setNextDrivingAction(action);
}
void DrivingActionManager::performNextAction() {
	if (head != nullptr && !isnan(this->chassis->getIMU()->getAngle())
			&& !isinf(this->chassis->getIMU()->getAngle())) {
		DrivingAction* new_head = head->getNextDrivingAction();

		if (head->getAction() == Action::DRIVE) {
			//check for the roadblock to make sure that it is safe
			float forwardDistance =
					this->chassis->getRangeFinder()->readSensorMM();

			/*if(forwardDistance < head->getValue() + 10) { //there is probably a road block
			 Serial.println("Roadblock detected");

			 Node *badNode = head->getEndNode();

			 badNode->deleteEdge(NORTH_INDEX);
			 badNode->deleteEdge(SOUTH_INDEX);
			 badNode->deleteEdge(WEST_INDEX);
			 badNode->deleteEdge(EAST_INDEX);

			 }*/
		}

		head->perform(this->chassis, turrent);

		if (new_head == nullptr) {
			scout(head->getEndNode());
		} else {
			head = new_head;
		}
	}
}

void DrivingActionManager::loop() {
	if (millis() - lastTime > 50) {
		lastTime = millis();
	} else {
		return;
	}

	if (chassis->getState() == DRIVING) {
		chassis->loop();
	} else if (turrent->getState() == TurrentState::SWEEP
			|| turrent->getState() == TurrentState::FIRE
			|| turrent->getState() == TurrentState::TURN_TURRENT) {
		turrent->loop();
	} else if (hasNext()) {
		performNextAction();
	}
}
void DrivingActionManager::pathToDrivingActions(Node *end) {
	std::vector<DrivingAction *> actions;

	Node *curr = end;

	//curr->printNode();

	while (curr != nullptr && curr->predecessor != nullptr) {
		Node::Edge *edge = graph->findConnectingEdge(curr->predecessor, curr);

		if (curr->getType() == MIDPOINT) {
			actions.insert(actions.begin(),
					new DrivingAction(Action::CHECK, 0, curr));
		}

		actions.insert(actions.begin(),
				new DrivingAction(DRIVE, edge->lengthX + edge->lengthY, curr));

		if (curr->predecessor->getEastEdge() == edge) {
			actions.insert(actions.begin(),
					new DrivingAction(TURN, 90, curr->predecessor));
		} else if (curr->predecessor->getWestEdge() == edge) {
			actions.insert(actions.begin(),
					new DrivingAction(TURN, 270, curr->predecessor));
		} else if (curr->predecessor->getNorthEdge() == edge) {
			actions.insert(actions.begin(),
					new DrivingAction(TURN, 360, curr->predecessor));
		} else if (curr->predecessor->getSouthEdge() == edge) {
			actions.insert(actions.begin(),
					new DrivingAction(TURN, 180, curr->predecessor));
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

	Node *end = this->graph->setBestPath(xStart, yStart, yEnd, xEnd);
	pathToDrivingActions(end);
}
void DrivingActionManager::scout(Node *startNode) {
	int currX = -1, currY = -1;

	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {
			if (graph->getNodeAt(x, y) != nullptr
					&& graph->getNodeAt(x, y) == startNode) {
				currX = x;
				currY = y;

				break;
			}
		}

		if (currX != -1) {
			break;
		}
	}

	bool found = false;
	for (int x = 0; x < 6 && !found; x++) {
		for (int y = 0; y < 6; y++) {
			if (graph->getNodeAt(x, y) != nullptr
					&& !graph->getNodeAt(x, y)->isChecked()
					&& graph->getNodeAt(x, y)->getType() == MIDPOINT) {
				found = true;
				break;
			}
		}
	}

	if (found) {
		int newX = -1, newY = -1;

		while (newX == -1) {
			int xIndex = rand() % 6;
			int yIndex = rand() % 6;

			if (graph->getNodeAt(xIndex, yIndex) != nullptr
					&& !graph->getNodeAt(xIndex, yIndex)->isChecked()
					&& graph->getNodeAt(xIndex, yIndex)->getType()
							== MIDPOINT) {
				newX = xIndex;
				newY = yIndex;

				break;
			}

		}

		/*Serial.println(
		 "scout(" + String(currX) + "," + String(currY) + ","
		 + String(newX) + "," + String(newY) + ")");*/
		this->setPath(currX, currY, newX, newY);
	} else {
		this->setPath(currX, currY, 0, 0);
	}

// need to move to the manager class

}

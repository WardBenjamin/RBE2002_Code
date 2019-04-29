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

		if (head->getAction() == Action::DRIVE
				&& this->chassis->getRangeFinder()->isRoadblock()) {
			Serial.println("[DrivingActionManager] Roadblock detected");

			Node *badNode = head->getEndNode();
			Node *startNode = badNode->predecessor;

			for (int x = 0; x < 4; x++) {
				if (startNode->edges[x] != nullptr
						&& startNode->edges[x]->destination == badNode) {
					startNode->deleteEdge(x); //delete the edge with the roadblock
					break;
				}
			}

			int *startCoords = graph->getNodeCoordinates(startNode);

			DrivingAction *endAction = head;

			while (endAction->getNextDrivingAction() != nullptr) {
				endAction = endAction->getNextDrivingAction();
			}

			int *endCoords = graph->getNodeCoordinates(endAction->getEndNode());

			Serial.println(
					"[DrivingActionManager] Recalculating path from ("
							+ String(startCoords[0]) + ", "
							+ String(startCoords[1]) + ") Node #"
							+ String(startNode->getID()) + " to ("
							+ String(endCoords[0]) + ", " + String(endCoords[1])
							+ ") Node #"
							+ String(endAction->getEndNode()->getID()));

			this->setPath(startCoords[0], startCoords[1], endCoords[0],
					endCoords[1]);

			return;
		}

		head->perform(this->chassis, turrent);
		this->chassis->getIMU()->addStreetAddress(
				(float) graph->getStreetName(head->getEndNode()));

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
			|| turrent->getState() == TurrentState::FIRE) {
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

			int *coords = graph->getNodeCoordinates(end);

			if (coords[0] % 2 == 0) {
				actions.insert(actions.begin(),
						new DrivingAction(Action::TURN, 270, curr));

				actions.insert(actions.begin(),
						new DrivingAction(Action::CHECK, 0, curr));

				actions.insert(actions.begin(),
						new DrivingAction(Action::TURN, 90, curr));

				actions.insert(actions.begin(),
						new DrivingAction(Action::CHECK, 0, curr));
			} else {
				actions.insert(actions.begin(),
						new DrivingAction(Action::TURN, 0, curr));

				actions.insert(actions.begin(),
						new DrivingAction(Action::CHECK, 0, curr));

				actions.insert(actions.begin(),
						new DrivingAction(Action::TURN, 180, curr));

				actions.insert(actions.begin(),
						new DrivingAction(Action::CHECK, 0, curr));
			}

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
					new DrivingAction(TURN, 0, curr->predecessor));
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
	this->head = new DrivingAction(Action::DRIVE, 0, graph->getNodeAt(xStart, yStart));

	Serial.println("[DrivingActionManager] setPath(" + String(xStart) + ", " + String(yStart) + ", " + String(xEnd) + ", " + String(yEnd) + ")");
	Serial.println("\t-> From Node #" + String(graph->getNodeAt(xStart, yStart)->getID()) + " to Node #" + String(graph->getNodeAt(xEnd, yEnd)->getID()));

	Node *end = this->graph->setBestPath(xStart, yStart, xEnd, yEnd);
	pathToDrivingActions(end);
}
void DrivingActionManager::scout(Node *startNode) {
	int currX = -1, currY = -1;

	int *coords = graph->getNodeCoordinates(startNode);

	currX = coords[0];
	currY = coords[1];

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

		Serial.println(
		 "scout(" + String(currX) + "," + String(currY) + ","
		 + String(newX) + "," + String(newY) + ")");
		this->setPath(currX, currY, newX, newY);
	} else {
		this->setPath(currX, currY, 0, 0);
	}

// need to move to the manager class

}

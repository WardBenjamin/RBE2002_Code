/*
 * Graph.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "Graph.h"

Graph::Graph() {
	nodeID = 0;

	createGraph();
	printGraph(startNode);
}

Graph::~Graph() {}

void Graph::createGraph() {
	startNode = new Node;
	spawn(0, 0, startNode);
}
void Graph::spawn(int x, int y, Node *node) {
	Node *newNode = nullptr;

	switch (x) {
	case 0:
		newNode = node->createNewEasternNode(START_TO_INTESECTION_DIST);
		break;
	case 1:
		newNode = node->createNewEasternNode(LENGTH_BETWEEN_TO_INTERSECTIONS);
		break;
	case 2:
		newNode = node->createNewEasternNode(FINAL_ROAD_DIST);
		break;
	}

	if (newNode != nullptr) {
		newNode->setID(nodeID);
		nodeID++;

		spawn(x + 1, y, newNode);
	}

	newNode = nullptr;

	switch (y) {
	case 0:
		newNode = node->createNewSouthernNode(START_TO_INTESECTION_DIST);
		break;
	case 1:
		newNode = node->createNewSouthernNode(LENGTH_BETWEEN_TO_INTERSECTIONS);
		break;
	case 2:
		newNode = node->createNewSouthernNode(FINAL_ROAD_DIST);
		break;
	}

	if (newNode != nullptr) {
		newNode->setID(nodeID);
		nodeID++;

		spawn(x, y + 1, newNode);
	}
}
Node* Graph::getNodeAt(int x, int y) {
	Node *curr = startNode;

	for (int i = 0; i < x; i++) {
		curr = curr->getEastEdge()->destination;
	}

	for (int j = 0; j < y; j++) {
		curr = curr->getSouthEdge()->destination;
	}

	return curr;
}
void Graph::setBestPath(DrivingActionManager *manager, int startX, int startY,
		int endX, int endY) {
	resetGraphCost();

	std::vector<Node*> openList;
	std::vector<Node*> closedList;

	openList.insert(openList.begin(), getNodeAt(startX, startY));

	while (!openList.empty()) {
		Node *lowestCost = nullptr;
		int index = 0;
		for (int x = 0; x < openList.size(); x++) {
			if (lowestCost == nullptr || lowestCost->f > openList.at(x)->f) {
				lowestCost = openList.at(x);
				index = x;
			}
		}

		openList.erase(openList.begin() + index);
		closedList.insert(closedList.begin(), lowestCost);

		if (lowestCost == getNodeAt(endX, endY)) {
			pathToDrivingActions(lowestCost, manager);
		}

		for (int x = 0; x < 4; x++) {
			bool found = false;
			for (int i = 0; i < closedList.size(); i++) {
				if (closedList.at(i) == lowestCost->edges[x]->destination) {
					found = true;
					break;
				}
				if (found) {
					continue;
				}
			}

			Node *child = lowestCost->edges[x]->destination;
			child->g = lowestCost->g
					+ sqrt(
							lowestCost->edges[x]->lengthX
									* lowestCost->edges[x]->lengthX
									+ lowestCost->edges[x]->lengthY
											* lowestCost->edges[x]->lengthY);
			child->h = 0; //change later for true implementation
			child->f = child->g + child->h;

			for (int i = 0; i < openList.size(); i++) {
				if (openList.at(i) == child) {
					found = true;
					break;
				}
				if (found && child->g > lowestCost->g) {
					continue;
				}
			}

			child->predecessor = lowestCost;
			openList.insert(openList.begin(), child);

		}

	}

}
void Graph::pathToDrivingActions(Node *end, DrivingActionManager *manager) {
	std::vector<DrivingAction *> actions;

	Node *curr = end;

	while(curr != nullptr) {
		Node::Edge *edge = findConnectingEdge(curr->predecessor, curr);
		actions.insert(actions.begin(), new DrivingAction(DRIVE, edge->lengthX + edge->lengthY));

		if(curr->getEastEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 0));
		} else if(curr->getWestEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 180));
		} else if(curr->getNorthEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 270));
		} if(curr->getSouthEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 90));
		}
		curr = curr->predecessor;
	}

	for(int x = 0; x < actions.size(); x++) {
		manager->addDrivingAction(actions.at(x)); //add all of the items to the paths

		Serial.print(String(x) + ". ");

		if(actions.at(x)->getAction() == TURN) {
			Serial.print("TURN to " + String(actions.at(x)->getValue()) + " degrees\n");
		} else {
			Serial.print("DRIVE for " + String(actions.at(x)->getValue()) + " mm\n");
		}

	}
}

Node::Edge* findConnectingEdge(Node *source, Node *destination) {
	for(int x = 0; x < 4; x++) {
		if(source->edges[x]->destination == destination && destination != nullptr) {
			return source->edges[x];
		}
	}
	return nullptr;
}


void Graph::resetGraphCost() {
	for (int x = 0; x < 4; x++) {
		for (int y = 0; y < 4; y++) {
			getNodeAt(x, y)->resetCost();
			getNodeAt(x, y)->resetPredecessor();
		}
	}
}
void Graph::printGraph(Node *node) {
	Serial.println("Node #" + String(node->getID()) + ":");

	if(node->getNorthEdge() != nullptr)
		Serial.println("\t- side: NORTH; length: " + String(node->getNorthEdge()->lengthY) + "; destination: Node #" + String(node->getNorthEdge()->destination->getID()));
	if(node->getSouthEdge() != nullptr)
			Serial.println("\t- side: SOUTH; length: " + String(node->getSouthEdge()->lengthY) + "; destination: Node #" + String(node->getSouthEdge()->destination->getID()));
	if(node->getWestEdge() != nullptr)
			Serial.println("\t- side: WEST; length: " + String(node->getWestEdge()->lengthX) + "; destination: Node #" + String(node->getWestEdge()->destination->getID()));
	if(node->getEastEdge() != nullptr)
			Serial.println("\t- side: EAST; length: " + String(node->getEastEdge()->lengthX) + "; destination: Node #" + String(node->getEastEdge()->destination->getID()));

	Serial.println();

	for(int x = 0; x < 4; x++) {
		printGraph(node->edges[x]->destination);
	}
}

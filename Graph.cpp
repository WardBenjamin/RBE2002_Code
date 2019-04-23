/*
 * Graph.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "Graph.h"

Graph::Graph() {
	createGraph();
	printGraph(0, 0);
}

Graph::~Graph() {
}

void Graph::createGraph() {
	for (int x = 0; x < 4; x++) {

		for (int y = 0; y < 4; y++) {
			nodeMap[x][y] = new Node();

			if (x > 0 && x < 3) {
				Node::Edge *east = new Node::Edge;
				east->source = nodeMap[x - 1][y];
				east->destination = nodeMap[x][y];

				Node::Edge *west = new Node::Edge;
				west->source = nodeMap[x][y];
				west->destination = nodeMap[x - 1][y];

				if (x == 1) {
					east->lengthX = START_TO_INTESECTION_DIST;
				} else if (x == 2) {
					east->lengthX = LENGTH_BETWEEN_TO_INTERSECTIONS;
				} else {
					east->lengthX = LEAD_ROADS_LENGTH;
				}

				west->lengthX = east->lengthX;

				nodeMap[x - 1][y]->setEasternEdge(east);
				nodeMap[x][y]->setWesternEdge(west);
			}
			if (y > 0 && y < 3) {
				Node::Edge *south = new Node::Edge;
				south->source = nodeMap[x][y - 1];
				south->destination = nodeMap[x][y];

				Node::Edge *north = new Node::Edge;
				north->source = nodeMap[x][y];
				north->destination = nodeMap[x][y - 1];

				if (y == 1) {
					south->lengthY = START_TO_INTESECTION_DIST;
				} else if (y == 2) {
					south->lengthY = LENGTH_BETWEEN_TO_INTERSECTIONS;
				} else {
					south->lengthY = LEAD_ROADS_LENGTH;
				}

				north->lengthY = south->lengthY;

				nodeMap[x][y - 1]->setSouthernEdge(south);
				nodeMap[x][y]->setNorthernEdge(north);
			}
		}
	}

	startNode = nodeMap[0][0];

}
Node* Graph::getNodeAt(int x, int y) {
	return nodeMap[x][y];
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

	while (curr != nullptr) {
		Node::Edge *edge = findConnectingEdge(curr->predecessor, curr);
		actions.insert(actions.begin(),
				new DrivingAction(DRIVE, edge->lengthX + edge->lengthY));

		if (curr->getEastEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 0));
		} else if (curr->getWestEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 180));
		} else if (curr->getNorthEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 270));
		}
		if (curr->getSouthEdge() == edge) {
			actions.insert(actions.begin(), new DrivingAction(TURN, 90));
		}
		curr = curr->predecessor;
	}

	for (int x = 0; x < actions.size(); x++) {
		manager->addDrivingAction(actions.at(x)); //add all of the items to the paths

		Serial.print(String(x) + ". ");

		if (actions.at(x)->getAction() == TURN) {
			Serial.print(
					"TURN to " + String(actions.at(x)->getValue())
							+ " degrees\n");
		} else {
			Serial.print(
					"DRIVE for " + String(actions.at(x)->getValue()) + " mm\n");
		}

	}
}

Node::Edge* Graph::findConnectingEdge(Node *source, Node *destination) {
	for (int x = 0; x < 4; x++) {
		if (source->edges[x]->destination == destination
				&& destination != nullptr) {
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
void Graph::printGraph(int x, int y){
	Node *node = getNodeAt(x, y);
	Serial.println("Node #" + String(node->getID()) + ":");

	if (node->getNorthEdge() != nullptr)
		Serial.println(
				"\t- side: NORTH; length: "
						+ String(node->getNorthEdge()->lengthY)
						+ "; destination: Node #"
						+ String(node->getNorthEdge()->destination->getID()));
	if (node->getSouthEdge() != nullptr)
		Serial.println(
				"\t- side: SOUTH; length: "
						+ String(node->getSouthEdge()->lengthY)
						+ "; destination: Node #"
						+ String(node->getSouthEdge()->destination->getID()));
	if (node->getWestEdge() != nullptr)
		Serial.println(
				"\t- side: WEST; length: "
						+ String(node->getWestEdge()->lengthX)
						+ "; destination: Node #"
						+ String(node->getWestEdge()->destination->getID()));
	if (node->getEastEdge() != nullptr)
		Serial.println(
				"\t- side: EAST; length: "
						+ String(node->getEastEdge()->lengthX)
						+ "; destination: Node #"
						+ String(node->getEastEdge()->destination->getID()));

	Serial.println();

	if(x < 3) {
		return printGraph(x + 1, y);
	} else if(y < 3) {
		return printGraph(0, y + 1);
	}
}

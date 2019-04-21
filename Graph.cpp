/*
 * Graph.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "Graph.h"

Graph::Graph() {
	// TODO Auto-generated constructor stub

}

Graph::~Graph() {
	// TODO Auto-generated destructor stub
}

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

	if(newNode != nullptr) {
		spawn(x + 1, y, newNode);
	}

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
	if(newNode != nullptr) {
		spawn(x, y + 1, newNode);
	}
}
Node* Graph::getNodeAt(int x, int y) {
	Node *curr = startNode;

	for(int i = 0; i < x; i++) {
		curr = curr->getEastEdge()->destination;
	}

	for(int j = 0; j < y; j++) {
		curr = curr->getSouthEdge()->destination;
	}

	return curr;
}
void Graph::setBestPath(DrivingActionManager *manager, int startX, int startY, int endX, int endY) {
	resetGraphCost();

	std::vector<Node*> openList;
	std::vector<Node*> closedList;

	openList.insert(openList.begin(), getNodeAt(startX, startY));

	while(!openList.empty()) {
		Node *lowestCost = nullptr;
		for(int x = openList.begin(); x < openList.size(); x++) {
			if(lowestCost == nullptr || lowestCost->f > openList.at(x)->f) {
				lowestCost = openList.at(x)->f;
			}
		}
	}

}

void Graph::resetGraphCost() {
	for(int x = 0; x < 4; x++) {
		for(int y = 0; y < 4; y++) {
			getNodeAt(x, y)->resetCost();
		}
	}
}

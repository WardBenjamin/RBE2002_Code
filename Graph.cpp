/*
 * Graph.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "Graph.h"

Graph::Graph() {
	createGraph();
	//printGraph(0, 0);
}

Graph::~Graph() {
}

void Graph::createGraph() {
	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {

			if (x % 2 == 1 && y % 2 == 1) {
				nodeMap[x][y] = nullptr;
				continue;
			}

			if (x % 2 == 0 && y % 2 == 0) {
				nodeMap[x][y] = new Node(INTERSECTION);
			} else {
				nodeMap[x][y] = new Node(MIDPOINT);
			}

			if (x > 0 && nodeMap[x - 1][y] != nullptr) {
				Node::Edge *east = new Node::Edge;
				east->source = nodeMap[x - 1][y];
				east->destination = nodeMap[x][y];

				Node::Edge *west = new Node::Edge;
				west->source = nodeMap[x][y];
				west->destination = nodeMap[x - 1][y];

				if (x == 1 || x == 2) {
					east->lengthX = START_TO_INTESECTION_DIST / 2;
				} else if (x == 3 || x == 4) {
					east->lengthX = LENGTH_BETWEEN_TO_INTERSECTIONS / 2;
				} else {
					east->lengthX = LEAD_ROADS_LENGTH;
				}

				west->lengthX = east->lengthX;

				nodeMap[x - 1][y]->setEasternEdge(east);
				nodeMap[x][y]->setWesternEdge(west);
			}

			if (y > 0 && nodeMap[x][y - 1] != nullptr) {
				Node::Edge *south = new Node::Edge;
				south->source = nodeMap[x][y - 1];
				south->destination = nodeMap[x][y];

				Node::Edge *north = new Node::Edge;
				north->source = nodeMap[x][y];
				north->destination = nodeMap[x][y - 1];

				if (y == 1 || y == 2) {
					south->lengthY = START_TO_INTESECTION_DIST / 2;
				} else if (y == 3 || y == 4) {
					south->lengthY = LENGTH_BETWEEN_TO_INTERSECTIONS / 2;
				} else {
					south->lengthY = LEAD_ROADS_LENGTH;
				}

				north->lengthY = south->lengthY;

				nodeMap[x][y - 1]->setSouthernEdge(south);
				nodeMap[x][y]->setNorthernEdge(north);
			}

		}
	}
}
Node* Graph::getNodeAt(int x, int y) {
	return nodeMap[x][y];
}
Node* Graph::setBestPath(int startX, int startY, int endX, int endY) {
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

		//Serial.println("Lowest Cost: ");
		//lowestCost->printNode();

		openList.erase(openList.begin() + index);
		closedList.insert(closedList.begin(), lowestCost);

		//Serial.println("Node erased from openList and place into closedList");

		if (lowestCost == getNodeAt(endX, endY)) {
			return lowestCost;
		}

		for (int x = 0; x < 4; x++) {
			bool found = false;
			for (int i = 0; i < closedList.size(); i++) {
				if (lowestCost->edges[x] == nullptr
						|| closedList.at(i)
								== lowestCost->edges[x]->destination) {
					found = true;
					break;
				}

			}
			if (found) {
				continue;
			}

			//Serial.println("x: " + String(x) + " closedList size(): " + String(closedList.size()));

			Node *child = lowestCost->edges[x]->destination;
			child->g = lowestCost->g
					+ sqrt(
							lowestCost->edges[x]->lengthX
									* lowestCost->edges[x]->lengthX
									+ lowestCost->edges[x]->lengthY
											* lowestCost->edges[x]->lengthY);
			if (child->getType() == NodeType::MIDPOINT && child->isChecked()) {
				child->g += sqrt(
						lowestCost->edges[x]->lengthX
								* lowestCost->edges[x]->lengthX
								+ lowestCost->edges[x]->lengthY
										* lowestCost->edges[x]->lengthY);
			}

			child->h = 0; //change later for true implementation
			child->f = child->g + child->h;

			for (int i = 0; i < openList.size(); i++) {
				if (openList.at(i) == child) {
					found = true;
					break;
				}
			}

			if (found && child->g > lowestCost->g) {
				continue;
			}

			child->predecessor = lowestCost;
			openList.insert(openList.begin(), child);

		}

	}
	return nullptr;
}

Node::Edge* Graph::findConnectingEdge(Node *source, Node *destination) {
	for (int x = 0; x < 4; x++) {
		if (source->edges[x] != nullptr
				&& source->edges[x]->destination == destination
				&& destination != nullptr) {
			return source->edges[x];
		}
	}
	return nullptr;
}

void Graph::resetGraphCost() {
	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {

			if (nodeMap[x][y] != nullptr) {
				getNodeAt(x, y)->resetCost();
				getNodeAt(x, y)->resetPredecessor();
			}
		}
	}
}
void Graph::printGraph(int x, int y) {
	Serial.println("printGraph() called");
	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {
			if (x % 2 == 1 && y % 2 == 1) {
				continue;
			}
			if (nodeMap[x][y] == nullptr) {
				Serial.println(
						"(" + String(x) + ", " + String(y) + "): nullptr");
			} else {
				nodeMap[x][y]->printNode();
			}
		}
	}
}
int* Graph::getNodeCoordinates(Node *node) {
	int *coords = nullptr;

	for (int x = 0; x < 6; x++) {
		for (int y = 0; y < 6; y++) {
			if (nodeMap[x][y] != nullptr && node != nullptr
					&& node == nodeMap[x][y]) {
				coords = new int[2];
				coords[0] = x;
				coords[1] = y;

				return coords;
			}
		}
	}

	return coords;
}

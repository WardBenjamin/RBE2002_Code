/*
 * Node.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "Node.h"

int Node::nodeID = 0;

Node::Node(NodeType type) {
	id = nodeID;
	nodeID++;

	edges[0] = nullptr;
	edges[1] = nullptr;
	edges[2] = nullptr;
	edges[3] = nullptr;

	this->type = type;

	this->predecessor = nullptr;
	this->checked = false;

	this->f = 0;
	this->g = 0;
	this->h = 0;
}

Node::~Node() {
	// TODO Auto-generated destructor stub
}
void Node::setEasternEdge(Edge *edge) {
	this->edges[EAST_INDEX] = edge;
}
void Node::setWesternEdge(Edge *edge) {
	this->edges[WEST_INDEX] = edge;
}
void Node::setNorthernEdge(Edge *edge) {
	this->edges[NORTH_INDEX] = edge;
}
void Node::setSouthernEdge(Edge *edge) {
	this->edges[SOUTH_INDEX] = edge;
}
void Node::setID(int id) {
	this->id = id;
}

void Node::printNode() {
	Serial.println("Node #" + String(getID()) + ":");

		if (getNorthEdge() != nullptr)
			Serial.println(
					"\t- side: NORTH; length: "
							+ String(getNorthEdge()->lengthY)
							+ "; destination: Node #"
							+ String(getNorthEdge()->destination->getID()));
		if (getSouthEdge() != nullptr)
			Serial.println(
					"\t- side: SOUTH; length: "
							+ String(getSouthEdge()->lengthY)
							+ "; destination: Node #"
							+ String(getSouthEdge()->destination->getID()));
		if (getWestEdge() != nullptr)
			Serial.println(
					"\t- side: WEST; length: "
							+ String(getWestEdge()->lengthX)
							+ "; destination: Node #"
							+ String(getWestEdge()->destination->getID()));
		if (getEastEdge() != nullptr)
			Serial.println(
					"\t- side: EAST; length: "
							+ String(getEastEdge()->lengthX)
							+ "; destination: Node #"
							+ String(getEastEdge()->destination->getID()));


		if(this->predecessor != nullptr) {
			Serial.println("predecessor: Node #" + String(this->predecessor->getID()));
		} else {
			Serial.println("predecessor: none");
		}
		Serial.println();
}

void Node::deleteEdge(int index) {
	if(this->edges[index] != nullptr) {
		Node *destination = this->edges[index]->destination;
		this->edges[index] = nullptr;

		if(index == SOUTH_INDEX) {
			destination->deleteEdge(NORTH_INDEX);
		} else if(index == NORTH_INDEX) {
			destination->deleteEdge(SOUTH_INDEX);
		} else if(index == EAST_INDEX) {
			destination->deleteEdge(WEST_INDEX);
		} else if(index == WEST_INDEX) {
			destination->deleteEdge(EAST_INDEX);
		}

	}
}

void Node::resetCost() {
	g = 0;
	h = 0;
	f = 0;
}
void Node::resetPredecessor() {
	predecessor = nullptr;
}
void Node::setChecked() {
	this->checked = true;
}

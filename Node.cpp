/*
 * Node.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "Node.h"

int Node::nodeID = 0;

Node::Node() {
	id = nodeID;
	nodeID++;

	edges[0] = nullptr;
	edges[1] = nullptr;
	edges[2] = nullptr;
	edges[3] = nullptr;
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

Node* Node::createNewEasternNode(float length) {
	Edge *edge = new Edge;
	edge->lengthX = length;
	edge->lengthY = 0;

	Node *node = new Node();

	edge->source = this;
	edge->destination = node;

	this->setEasternEdge(edge);

	edge = new Edge;
	edge->lengthX = length;
	edge->lengthY = 0;

	edge->source = node;
	edge->destination = this;

	node->setWesternEdge(edge);

	return node;
}
Node* Node::createNewSouthernNode(float length) {
	Edge *edge = new Edge;
	edge->lengthY = length;
	edge->lengthX = 0;

	Node *node = new Node();

	edge->source = this;
	edge->destination = node;

	this->setSouthernEdge(edge);

	edge = new Edge;
	edge->lengthY = length;
	edge->lengthX = 0;

	edge->source = node;
	edge->destination = this;

	node->setNorthernEdge(edge);

	return node;
}

void Node::resetCost() {
	g = 0;
	h = 0;
	f = 0;
}
void Node::resetPredecessor() {
	predecessor = nullptr;
}

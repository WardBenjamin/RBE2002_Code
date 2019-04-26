/*
 * Graph.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef GRAPH_H_
#define GRAPH_H_


#include "Node.h"
#include <cmath>
#include "CityGeometry.h"
#include <vector>

class Graph {
private:
	Node *startNode;

	Node *nodeMap[6][6];
public:
	Graph();
	virtual ~Graph();

	void createGraph();

	Node* getNodeAt(int, int);

	int* getNodeCoordinates(Node *);

	Node* setBestPath(int, int, int, int);
	Node::Edge* findConnectingEdge(Node*, Node*);

	void resetGraphCost();
	void printGraph(int, int);


	String getNodeStreetAddress(Node*, int);

};

#endif /* GRAPH_H_ */

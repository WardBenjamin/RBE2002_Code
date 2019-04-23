/*
 * Graph.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include "Node.h"
#include "CityGeometry.h"
#include "DrivingActionManager.h"
#include <vector>

class Graph {
private:
	Node *startNode;

	Node *nodeMap[4][4];
public:
	Graph();
	virtual ~Graph();

	void createGraph();

	Node* getNodeAt(int, int);

	void setBestPath(DrivingActionManager*, int, int, int, int);

	void pathToDrivingActions(Node*, DrivingActionManager*);
	Node::Edge* findConnectingEdge(Node*, Node*);


	void resetGraphCost();
	void printGraph(int, int);
};

#endif /* GRAPH_H_ */

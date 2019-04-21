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
public:
	Graph();
	virtual ~Graph();

	void createGraph();
	void spawn(int x, int y, Node *node);

	Node* getNodeAt(int, int);

	void setBestPath(DrivingActionManager*, int, int, int, int);

	void resetGraphCost();
};

#endif /* GRAPH_H_ */

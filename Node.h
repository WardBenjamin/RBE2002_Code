/*
 * Node.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef NODE_H_
#define NODE_H_

#define NORTH_INDEX 0
#define EAST_INDEX 1
#define WEST_INDEX 2
#define SOUTH_INDEX 3

#include <Arduino.h>

enum NodeType {
	INTERSECTION, MIDPOINT
};

class Node {
private:
	int id;
	bool checked;

	NodeType type;
public:
	struct Edge {
		float lengthX = 0, lengthY = 0;
		Node *source, *destination;
	};

	static int nodeID;

	Node(NodeType);
	virtual ~Node();
	bool blocked = false;

	Edge *edges[4];

	Edge* getNorthEdge() const {
		return edges[NORTH_INDEX];
	}
	Edge* getSouthEdge() const {
		return edges[SOUTH_INDEX];
	}
	Edge* getWestEdge() const {
		return edges[WEST_INDEX];
	}
	Edge* getEastEdge() const {
		return edges[EAST_INDEX];
	}

	void setEasternEdge(Edge*);
	void setWesternEdge(Edge*);
	void setSouthernEdge(Edge*);
	void setNorthernEdge(Edge*);

	float g, h, f;
	Node *predecessor;

	void resetCost();
	void resetPredecessor();

	void deleteEdge(int);

	void printNode();

	void setID(int);
	int getID() const {
		return id;
	}

	NodeType getType() const {
		return type;
	}

	bool isChecked() const {
		return checked;
	}
	void setChecked();
};

#endif /* NODE_H_ */

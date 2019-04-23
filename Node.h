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


class Node {
private:
	int id;
public:
	struct Edge {
		float lengthX, lengthY;
		Node *source, *destination;
	};

	static int nodeID;

	Node();
	virtual ~Node();

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

	Node* createNewSouthernNode(float);
	Node* createNewEasternNode(float);

	float g, h, f;
	Node *predecessor;

	void resetCost();
	void resetPredecessor();

	void setID(int);
	int getID() const {
		return id;
	}
};

#endif /* NODE_H_ */

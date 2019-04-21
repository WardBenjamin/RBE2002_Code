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
	struct Edge {
		float lengthX, lengthY;
		Node *source, destination;
	};

	Edge *edges[4];
public:
	Node();
	virtual ~Node();

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
		return edges[NORTH_INDEX];
	}

	void setEasternEdge(Edge*);
	void setWesternEdge(Edge*);
	void setSouthernEdge(Edge*);
	void setNorthernEdge(Edge*);

	Node* createNewSouthernNode(float);
	Node* createNewEasternNode(float);

	float g, h, f;

	void resetCost();
};


#endif /* NODE_H_ */

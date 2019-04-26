/*
 * DrivingAction.h
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#ifndef DRIVINGACTION_H_
#define DRIVINGACTION_H_

#include "DrivingChassis.h"
#include "Node.h"
#include "Turrent.h"
#include "Arduino.h"


#define MS_PER_DEGREE 40
#define MS_PER_MILIMETER 20

enum Action {
	DRIVE = 0, TURN = 1, CHECK = 2
};

class DrivingAction {

private:
	Action action;
	float param;

	DrivingAction *next;

	Node *endNode;
public:
	DrivingAction(Action, float, Node*);
	virtual ~DrivingAction();


	void setNextDrivingAction(DrivingAction*);
	DrivingAction* getNextDrivingAction();

	void perform(DrivingChassis*, Turrent*);

	void printDrivingAction();

	Action getAction() const {
		return action;
	}
	float getValue() const {
		return param;
	}

	Node* getEndNode() const {
		return endNode;
	}
};

#endif /* DRIVINGACTION_H_ */

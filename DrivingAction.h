/*
 * DrivingAction.h
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#ifndef DRIVINGACTION_H_
#define DRIVINGACTION_H_

#include "DrivingChassis.h"

#define MS_PER_DEGREE 10
#define MS_PER_MILIMETER 10

enum Action {
	DRIVE, TURN
};

class DrivingAction {

private:
	Action action;
	float param;

	DrivingAction *next;
public:
	DrivingAction(Action, float);
	virtual ~DrivingAction();


	void setNextDrivingAction(DrivingAction*);
	DrivingAction* getNextDrivingAction();

	void perform(DrivingChassis*);
};

#endif /* DRIVINGACTION_H_ */

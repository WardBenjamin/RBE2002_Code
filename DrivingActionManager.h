/*
 * DrivingActionManager.h
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#ifndef DRIVINGACTIONMANAGER_H_
#define DRIVINGACTIONMANAGER_H_

#include "DrivingAction.h"
#include "DrivingChassis.h"

class DrivingActionManager {

private:
	DrivingAction *head;
	DrivingChassis *chassis;
public:
	DrivingActionManager(DrivingChassis *chassis);
	virtual ~DrivingActionManager();

	bool hasNext();
	void addDrivingAction(DrivingAction*);
	void performNextAction();
};

#endif /* DRIVINGACTIONMANAGER_H_ */

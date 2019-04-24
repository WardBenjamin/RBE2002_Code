/*
 * DrivingActionManager.h
 *
 *  Created on: Apr 20, 2019
 *      Author: developer
 */

#ifndef DRIVINGACTIONMANAGER_H_
#define DRIVINGACTIONMANAGER_H_

#include "DrivingAction.h"
#include "Graph.h"
#include <math.h>
#include "Turrent.h"



class DrivingActionManager {
private:
	DrivingAction *head;
	DrivingChassis *chassis;

	Graph *graph;

	Turrent *turrent;

	long lastTime = 0;

	void pathToDrivingActions(Node*);
public:
	DrivingActionManager(DrivingChassis*, Turrent*);
	virtual ~DrivingActionManager();

	bool hasNext();
	void addDrivingAction(DrivingAction*);
	void performNextAction();

	void loop();

	void scout(Node *);

	void setPath(int, int, int, int);


	Graph* getGraph() const {
		return graph;
	}
};

#endif /* DRIVINGACTIONMANAGER_H_ */

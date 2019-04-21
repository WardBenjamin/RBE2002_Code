/*
 * CityNavigator.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef CITYNAVIGATOR_H_
#define CITYNAVIGATOR_H_

#include "City.h"
#include "DrivingActionManager.h"

class CityNavigator {
private:
	City *city;
	DrivingActionManager *manager;
public:
	CityNavigator(City*, DrivingActionManager*);
	virtual ~CityNavigator();

	void findBestPath(int, int);
};

#endif /* CITYNAVIGATOR_H_ */

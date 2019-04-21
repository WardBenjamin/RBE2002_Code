/*
 * City.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef CITY_H_
#define CITY_H_

#include "Street.h"
#include "RobotGeometry.h"
#include "CityGeometry.h"

class City {

private:
	void generateCity();
	Street *startNode; // south street, east street
public:
	City();
	virtual ~City();


	void printCity();
};

#endif /* CITY_H_ */

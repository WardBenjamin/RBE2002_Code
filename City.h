/*
 * City.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef CITY_H_
#define CITY_H_

#include "Street.h"

class City {

private:
	void generateCity();
	Street *startNode[2]; // south street, east street
public:
	City();
	virtual ~City();


	void printCity();
};

#endif /* CITY_H_ */

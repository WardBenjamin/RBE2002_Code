/*
 * City.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "City.h"

City::City() {
	// TODO Auto-generated constructor stub

}

City::~City() {
	// TODO Auto-generated destructor stub
}

void City::generateCity() {

	float currX = STARTING_ROBOT_X;
	float currY = STARTING_ROBOT_Y;

	Street **last_column = nullptr;
	Street *column[3];

	for(int x = 0;  x < 3; x++) {
		column = new Street[3];

		column[0] = new Street("", currX, currY, currX, currY + START_TO_INTESECTION_DIST);
		currY += START_TO_INTESECTION_DIST;

		column[1] = new Street("", currX, currY, currX, currY + LENGTH_BETWEEN_TO_INTERSECTIONS);
		column[0]->setSouthIntersection(column[1]);
		currY += LENGTH_BETWEEN_TO_INTERSECTIONS;

		column[2] = new Street("", currX, currY, currX, currY + FINAL_ROAD_DIST);


		if(last_column != nullptr) {
			Street *start = new Street("", last_column[0]->getStartX(), last_column[0]->getStartY(), column[0]->getStartX(), column[0]->getStartY());
			start->setWestIntersection(last_column[0]);
			start->setEastIntersection(column[0]);


			Street *start = new Street("", last_column[0]->getStartX(), last_column[0]->getStartY(), column[0]->getStartX(), column[0]->getStartY());
			start->setWestIntersection(last_column[0]);
			start->setEastIntersection(column[0]);

		}

		last_column = column;

		if(x == 0) {
			currX +=  START_TO_INTESECTION_DIST;
		} else if(x == 1) {
			currX += LENGTH_BETWEEN_TO_INTERSECTIONS;
		} else if(x == 2) {
			currY += FINAL_ROAD_DIST;
		}
	}


}


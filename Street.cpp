/*
 * Street.cpp
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#include "Street.h"

Street::Street(char name[12], float startX, float startY, float endX, float endY) {
	for(int x = 0; x < 12; x++) {
		this->name[x] = name[x];
	}

	this->startX = startX;
	this->startY = startY;
	this->endX = endX;
	this->endY = endY;


	this->length = endX - startX;
	this->height = endY - startY;
}

Street::~Street() {

}

float Street::getGlobalPIDYSetPoint() {
	 return endY;
}
float Street::getGlobalPIDXSetPoint() {
	return endX;
}
float Street::getLocalPIDYSetPoint() {
	return endY - startY;
}
float Street::getLocalPIDXSetPoint() {
	return endX - startX;
}



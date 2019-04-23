/*
 * RangeFinder.cpp
 *
 *  Created on: Apr 23, 2019
 *      Author: developer
 */

#include "RangeFinder.h"

RangeFinder::RangeFinder(int analogPin) {
	this->analogPin = analogPin;
}

RangeFinder::~RangeFinder() {}

float RangeFinder::readSensorMM() {
	float reading = analogRead(this->analogPin);
	reading *= (512) / (4096);
	reading *= 25.4;
	return reading;
}


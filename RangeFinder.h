/*
 * RangeFinder.h
 *
 *  Created on: Apr 23, 2019
 *      Author: developer
 */

#ifndef RANGEFINDER_H_
#define RANGEFINDER_H_

#include "Arduino.h"
#include <driver/adc.h>

class RangeFinder {
private:
	int analogPin;
public:
	RangeFinder(int);
	virtual ~RangeFinder();

	float readSensorMM();
	bool isRoadblock();
};

#endif /* RANGEFINDER_H_ */

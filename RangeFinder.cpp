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

bool RangeFinder::isRoadblock() {
	 adc1_config_width(ADC_WIDTH_BIT_12);
	 adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db);
	 float sum = 0;

	 for(int x = 0; x < 10; x++) {
		 sum += adc1_get_raw(ADC1_CHANNEL_0);
	 }

	 sum /= 10.0;


	 Serial.println("[DrivingActionManager] Range Detected: " + String(sum));

	 return sum == 0;
}


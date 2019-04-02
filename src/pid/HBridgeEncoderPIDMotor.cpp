/*
 * HBridgeEncoderPIDMotor.cpp
 *
 *  Created on: Oct 17, 2018
 *      Author: hephaestus
 */

#include "HBridgeEncoderPIDMotor.h"

HBridgeEncoderPIDMotor::HBridgeEncoderPIDMotor() {
	// TODO Auto-generated constructor stub
	this->directionPin = -1;
	setOutputBoundingValues(-HBRIDGE_MAX, HBRIDGE_MAX, 0, HBRIDGE_DEADBAND, HBRIDGE_DEADBAND,
			16.0 * // Encoder CPR
			50.0 * // Motor Gear box ratio
			1.0 * // motor to wheel stage ratio
			(1.0 / 360.0) * // degrees per revolution
			encoder.countsMode,
			186.0 * 60.0 * 360.0,0);
}

HBridgeEncoderPIDMotor::~HBridgeEncoderPIDMotor() {
	// TODO Auto-generated destructor stub
}

void HBridgeEncoderPIDMotor::attach(int pwmPin, int directionPin, int encoderA,
		int encoderB) {
	encoder.attachHalfQuad(encoderA, encoderB);
	this->directionPin = directionPin;
	motor.attachPin(pwmPin, 10000, 8);
	pidinit();
	pinMode(directionPin, OUTPUT);
	setOutput(0);
}
int64_t HBridgeEncoderPIDMotor::getPosition() {
	return encoder.getCount();
}

void HBridgeEncoderPIDMotor::setOutput(int32_t out) {
	if (out > 0) {
		digitalWrite(directionPin, HIGH); // turn the LED on (HIGH is the voltage level)
	} else {
		digitalWrite(directionPin, LOW);
	}
	int myOut = abs(out);
	if (myOut < getOutputMaxDeadbad() && out != 0)
		myOut = getOutputMaxDeadbad();

	motor.write(myOut);
}
void HBridgeEncoderPIDMotor::overrideCurrentPositionHardware(int64_t val) {
	encoder.setCount(val);
}
//This function should analogRead the current sense from the motor driver
//and convert the value to current in milliamps
double HBridgeEncoderPIDMotor::calcCur(void) {

	const float capacitance = 1E-9, resistance = 330;

	unsigned short int value = analogRead(36); // read from analog sense pin. I would of used constant defined in config.h but keep getting some linking error
	float voltage = map(value, 0, 1024, 0, 3.3); //now we have the voltaged

	float impedance = resistance;

	float current = (voltage / impedance) * 1E-3;

	Serial.println("analogRead(): " + String(value) + " Current: " + String(current));
	return current;
}

/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "Arduino.h"
#include "RBEPID.h"

//Class constructor
RBEPID::RBEPID() {

}

//Function to set PID gain values
void RBEPID::setpid(float P, float I, float D) {
	kp = P;
	ki = I;
	kd = D;
}

/**
 * calc the PID control signel
 *
 * @param setPoint is the setpoint of the PID system
 * @param curPosition the current position of the plan
 * @return a value from -1.0 to 1.0 representing the PID control signal
 * @author Luke Trujillo
 * @date 4/3/2019
 */
float RBEPID::calc(double setPoint, double curPosition) {
	// calculate error
	float err = setPoint - curPosition;

	// calculate derivative of error
	float d_err = err - last_error;

	if (last_error != 0) {
		if (err / last_error < 0 && setPoint > 0) {
			clearIntegralBuffer(); //clear the integral
		} else if(setPoint < 0 && err / last_error > 0) {
			clearIntegralBuffer(); //clear the integral
		}
	}

	sum_error += err; //sum up the integral error

	last_error = err; //setup for the next derivative error

	float out = err * kp + kd * d_err + ki * sum_error;	// simple P controller

	if (out > 1) //the maximum output is 1
		out = 1;
	if (out < -1) // the minimum output it -1
		out = -1;

	return out; //return the output
}

/**
 * Clear the internal representation fo the integral term.
 *
 */
void RBEPID::clearIntegralBuffer() {
	sum_error = 0;
}


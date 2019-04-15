/*
 * DrivingChassis.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: hephaestus
 */

#include "DrivingChassis.h"

/**
 * Compute a delta in wheel angle to traverse a specific distance
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param distance a distance for this wheel to travel in MM
 * @return the wheel angle delta in degrees
 */
float DrivingChassis::distanceToWheelAngle(float distance) {
	return (distance * 360) / (2 * PI * this->mywheelRadiusMM); // LT - (4/9/2019)
}

/**
 * Compute the arch length distance the wheel needs to travel through to rotate the base
 * through a given number of degrees.
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param angle is the angle the base should be rotated by
 * @return is the linear distance the wheel needs to travel given the this CHassis's wheel track
 */
float DrivingChassis::chassisRotationToWheelDistance(float angle) {
	return ((2 * PI * this->mywheelTrackMM) / 2) * (angle / 360.0); // LT - (4/9/2019)
}

DrivingChassis::~DrivingChassis() {
	// do nothing
}

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * @param left the left motor
 * @param right the right motor
 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
 */
DrivingChassis::DrivingChassis(PIDMotor * left, PIDMotor * right,
		float wheelTrackMM, float wheelRadiusMM, GetIMU * imu) {

	this->IMU = imu;
	this->myleft = left;
	this->myright = right;
	this->mywheelTrackMM = wheelTrackMM;
	this->mywheelRadiusMM = wheelRadiusMM;

	this->xPID = new RBEPID();
	this->xPID->setpid(0.05, 0, 0);

	this->yPID = new RBEPID();
	this->yPID->setpid(0.01, 0, 0.01);

	this->anglePID = new RBEPID();
	this->anglePID->setpid(0.025, 0, 0);

	this->isYCorrectionMode = false;

	this->state = STANDBY;
}

/**
 * Start a drive forward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 * @note myleft->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration) {
	this->targetTime = msDuration;

	targetX = mmDistanceFromCurrent;
	targetY = 0;

	IMU->setXPosition(0);
	IMU->setYPosition(0);
	IMU->setZPosition(0);

	this->myleft->overrideCurrentPosition(0);
	this->myright->overrideCurrentPosition(0);

	state = DRIVING;

	targetVelocity = this->distanceToWheelAngle(mmDistanceFromCurrent)
			/ (msDuration / 1000);

	startTime = millis();

	isTurning = false;

	setAngleAdjustment(180);

	this->xPID->setpid(0.05, 0, 0);
	this->yPID->setpid(0.01, 0, 0.01);
	this->anglePID->setpid(0.025, 0, 0);
}

/**
 * Start a turn action
 *
 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
 * Positive angles should rotate to the left
 *
 * This rotation is a positive rotation about the Z axis of the robot.
 *
 * @param degreesToRotateBase the number of degrees to rotate
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 *  @note this function is fast-return and should not block
 *  @note myleft->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::turnDegrees(float degreesToRotateBase, int msDuration) {
	setAngleAdjustment(180);

	this->targetTime = msDuration;

	targetX = 0;
	targetY = 0;

	IMU->setXPosition(0);
	IMU->setYPosition(0);
	IMU->setZPosition(0);

	this->myleft->overrideCurrentPosition(0);
	this->myright->overrideCurrentPosition(0);

	state = DRIVING;

	this->targetAngle = 180 + degreesToRotateBase;

	startTime = millis();

	isTurning = true;

	anglePID->setpid(1, 0, 0.2);


}

/**
 * Check to see if the chassis is performing an action
 *
 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
 *
 *  @note this function is fast-return and should not block
 */
bool DrivingChassis::isChassisDoneDriving() {
	return false;
}

/**
 * This function should be called by StudentRobot's loop everytime and manage the setpoints of the robot.
 * @author Luke Trujillo
 *
 * @note This will most likely be where should implement Kevin's algorithm for Lab 4.
 */
void DrivingChassis::loop() {

	if (state == DRIVING) {

		Serial.println("\n\n\n<---loop(DRIVING)--->");
		Serial.println("heading: {" + String(targetX) + "," + String(targetY) + ", " + String(lastAngle) + "}\n");
		//step 1: determine the location
		float x = this->IMU->getXPosition();
		float y = this->IMU->getYPosition();

		float *change = this->trapzoid_approx(myleft->getPosition(), myright->getPosition());

		x += change[0];
		y += change[1];

		update(x, y);

		//2. figure out what corrections need to happen
		float elapsed = myright->myFmap(millis() - startTime, 0, targetTime, 0,
				1);

		float elapsed_time = millis() - startTime;

		float sineTerm = 1
				- ((cos(-PI * (elapsed_time / targetTime)) / 2) + 0.5); //this is wrong

		float sineTargetX = sineTerm * targetX;
		float sineTargetY = sineTerm * targetY;
		//float sineTargetAngle = sineTerm * 180

		if (elapsed == 1 || sineTerm == 1) { //need to add back sineTem term
			state = DONE;

			Serial.println("DONE!");

			myleft->stop();
			myright->stop();

			return;
		}


		Serial.println("calculate_PID():");
		//3. compute PID result
		float xOut = xPID->calc(sineTargetX, x);
		Serial.println("\txPID->calc(" + String(sineTargetX) + ", " + String(x) + ") = " + String(xOut));

		float yOut = yPID->calc(sineTargetY, y);
		Serial.println("\tyPID->calc(" + String(sineTargetY) + ", " + String(y) + ") = " + String(yOut));

		float angleOut = anglePID->calc(targetAngle, getAngle()); //remove angle toggle point by subtracting 150
		Serial.println("\tanglePID->calc(" + String(targetAngle) + ", " + String(getAngle()) + ") = " + String(angleOut));

		Serial.println();

		//4. choose y or theta correction term
		float upperLimit = 1, switchLimit = 0.1;

		float turningTerm = angleOut;

		if (!isYCorrectionMode && abs(abs(sineTargetY) - abs(y))> upperLimit && !isTurning) { //might be wrong

			Serial.println("Correction mode entered!");
			isYCorrectionMode = true;
		} else if (isYCorrectionMode
				&& abs(abs(sineTargetY) - abs(y)) < switchLimit) {
			isYCorrectionMode = false;
		}

		if (isYCorrectionMode) {
			turningTerm = -yOut;
		}

		float powerTerm = xOut;

		if(isTurning) {
			powerTerm *= 0;
		}

		float* powers = joystick_algorithm(powerTerm, turningTerm);


		myleft->setVelocityDegreesPerSecond(powers[0] * 300 * 0.75);
		myright->setVelocityDegreesPerSecond(powers[1] * -300);

		if(isTurning) {
			myright->setVelocityDegreesPerSecond(powers[1] * -300 * 0.8);
		}

		if(isYCorrectionMode) {
			Serial.println("YCorrectionMode = true");
		} else {
			Serial.println("YCorrectionMode = false");
		}

	}
}
float DrivingChassis::getAngle() {
	float angle = IMU->getEULER_azimuth() + adjustAngle;
	if (angle > 360)
		return angle - 360;
	if (angle < 0)
		return angle + 360;
	return angle;
}

void DrivingChassis::setAngleAdjustment(float angle) {
	adjustAngle = angle - IMU->getEULER_azimuth();
	lastAngle = getAngle();

	//targetAngle = angle;
}

float DrivingChassis::getDistanceFromTicks(float ticks) {
	return (ticks / 3200.0) * (2 * PI * this->mywheelRadiusMM);
}

float* DrivingChassis::trapzoid_approx(float leftMotorTicks, float rightMotorTicks) {
		float distanceLeft = leftMotorTicks - lastLeftEncoder;
		float distanceRight = rightMotorTicks - lastRightEncoder;

		float changeDistance = (abs(distanceRight) + abs(distanceLeft)) / 2;
		changeDistance = getDistanceFromTicks(changeDistance);

		float *change = new float[2];

		change[0] = changeDistance
				* cos(radians(180 - getAngle()));
		change[1] = changeDistance
				* sin(radians(180 - getAngle()));


		if(trapzoid_debug) {
			Serial.println("trapoid_approx(" + String(leftMotorTicks) + ", " + String(rightMotorTicks) + "):");
			Serial.println("\tlastAngle: " + String(lastAngle));
			Serial.println("\tlastEncoderValues: {" + String(lastLeftEncoder) + ", " + String(lastRightEncoder) + "}");
			Serial.println("\tchangeDistance:" + String(changeDistance));
			Serial.println("\tchange: {" + String(change[0]) + ", " + String(change[0]) + "}");
			Serial.println();
		}

		return change;
}
float* DrivingChassis::joystick_algorithm(float powerTerm, float turningTerm) {
	float *motorPowers = new float[2];
	motorPowers[0] = powerTerm + turningTerm;
	motorPowers[1] = powerTerm - turningTerm;


	float max = fmax(abs(motorPowers[0]), abs(motorPowers[1]));

	if (max > 1) {
		motorPowers[0] /= max;
		motorPowers[1] /= max;
	}


	if(joystick_debug) {
		Serial.println("joystick_algorithm(" + String(powerTerm) + ", " + String(turningTerm) + "):");
		Serial.println("\tmotorPowers: {" + String(motorPowers[0]) + ", " + String(motorPowers[1]) + "}");
		Serial.println();
	}

	return motorPowers;
}

void DrivingChassis::update(float x, float y) {
	IMU->setXPosition(x);
	IMU->setYPosition(y);

	lastLeftEncoder = myleft->getPosition();
	lastRightEncoder = myright->getPosition();
	lastAngle = getAngle(); //update the last angle

	if(showIMU) {
		Serial.println(
						"IMU: {" + String(IMU->getXPosition()) + ", "
								+ String(IMU->getYPosition()) + "}\n");
	}
}

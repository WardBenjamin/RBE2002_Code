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
	this->xPID->setpid(0.1, 0, 0);

	this->yPID = new RBEPID();
	this->yPID->setpid(0.1, 0, 0);

	this->anglePID = new RBEPID();
	this->anglePID->setpid(0.1, 0, 0);

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

	this->myleft->setVelocityDegreesPerSecond(targetVelocity);
	this->myright->setVelocityDegreesPerSecond(targetVelocity);


	setAngleAdjustment(180);

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
		//step 1: determine the location
		float x = this->IMU->getXPosition();
		float y = this->IMU->getYPosition();

		float distanceLeft = myleft->getAngleDegrees() - lastLeftEncoder;
		float distanceRight = myright->getAngleDegrees() - lastRightEncoder;

		float changeDistance = (abs(distanceRight) + abs(distanceLeft)) / 2;
		changeDistance *= (600.0 / 1180.0);

		float changeX = changeDistance
				* cos(radians(lastAngle));
		float changeY = changeDistance
				* sin(radians(lastAngle));

		lastLeftEncoder = myleft->getAngleDegrees();
		lastRightEncoder = myright->getAngleDegrees();
		lastAngle = getAngle(); //update the last angle

		IMU->setXPosition(x + changeX);
		IMU->setYPosition(y + changeY);

		Serial.println(
				"IMU: (" + String(IMU->getXPosition()) + ", "
						+ String(IMU->getYPosition()) + ")");

		//2. figure out what corrections need to happen
		float elapsed = myright->myFmap(millis() - startTime, 0, targetTime, 0,
				1);

		float elapsed_time = millis() - startTime;

		float sineTerm = 1
				- ((cos(-PI * (elapsed_time / targetTime)) / 2) + 0.5); //this is wrong

		float sineTargetX = sineTerm * targetX;
		float sineTargetY = sineTerm * targetY;

		/*if (timesLoop % 10 || timesLoop == 0) {
		 Serial.println("2. Figure out what needs to happen");
		 Serial.println("elapsed: " + String(elapsed));
		 Serial.println("sineTerm: " + String(sineTerm));
		 Serial.println("elapsed_time: " + String(elapsed_time));

		 Serial.println("\n\n-----\n\n");
		 }*/

		if (elapsed == 1 || sineTerm == 1) { //need to add back sineTem term
			state = DONE;

			myleft->stop();
			myright->stop();

			return;
		}

		//3. compute PID result
		float xOut = xPID->calc(sineTargetX, x + changeX);
		float yOut = yPID->calc(sineTargetY, y + changeY);
		float angleOut = anglePID->calc(180, getAngle()); //remove angle toggle point by subtracting 150

		//if (timesLoop % 10 || timesLoop == 0) {
		Serial.println("xOut: " + String(xOut));
		Serial.println("yOut: " + String(yOut));
		Serial.println("angleOut: " + String(angleOut));
		//}*/

		//4. choose y or theta correction term
		float upperLimit = 3, switchLimit = 0.5;

		float turningTerm = angleOut;

		if (!isYCorrectionMode && abs(targetY - (changeY + y)) > upperLimit) { //might be wrong

			Serial.println("Correction mode entered!");
			isYCorrectionMode = true;
		} else if (isYCorrectionMode
				&& abs(targetY - (changeY + y)) < switchLimit) {
			isYCorrectionMode = false;
		}

		if (isYCorrectionMode) {
			turningTerm = yOut;
		}

		float powerTerm = xOut;

		float leftPower = powerTerm + turningTerm;
		float rightPower = powerTerm - turningTerm;

		float max = fmax(abs(leftPower), abs(rightPower));

		if (max > 1) {
			leftPower /= max;
			rightPower /= max;
		}
		myleft->setVelocityDegreesPerSecond(leftPower * -400);
		myright->setVelocityDegreesPerSecond(rightPower * 400);

		Serial.println("Left Velocity: " + String(leftPower * -400));
		Serial.println("Right Velocity: " + String(rightPower * 400));
		Serial.println();

		//timesLoop++;
	}
}
float DrivingChassis::getAngle() {
	float angle = IMU->getEULER_azimuth() - adjustAngle;
	if (angle > 360)
		return angle - 360;
	if (angle < 0)
		return angle + 360;
	return angle;
}

void DrivingChassis::setAngleAdjustment(float angle) {
	adjustAngle = angle - IMU->getEULER_azimuth();
	lastAngle = getAngle();
}


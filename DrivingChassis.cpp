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
		float wheelTrackMM, float wheelRadiusMM,GetIMU * imu) {

	this->IMU = imu;
	this->myleft = left;
	this->myright = right;
	this->mywheelTrackMM = wheelTrackMM;
	this->mywheelRadiusMM = wheelRadiusMM;

	this->state = START;
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
	this->myleft->overrideCurrentPosition(0);
	this->myright->overrideCurrentPosition(0);

	this->myleft->setSetpoint(mmDistanceFromCurrent);
	this->myright->setSetpoint(mmDistanceFromCurrent);

	this->myleft->setVelocityDegreesPerSecond(distanceToWheelAngle(mmDistanceFromCurrent) / (msDuration / 1000));
	this->myright->setVelocityDegreesPerSecond(distanceToWheelAngle(mmDistanceFromCurrent) / (msDuration / 1000));
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
	return 0;
}

/**
 * This function should be called by StudentRobot's loop everytime and manage the setpoints of the robot.
 * @author Luke Trujillo
 *
 * @note This will most likely be where should implement Kevin's algorithm for Lab 4.
 */
void DrivingChassis::loop() {

	if(state == 0) {
		state = WORKING;
		this->myleft->setSetpoint(600);
		this->myright->setSetpoint(600);
	} else if(state == 1) {
		//step 1 update absolute position of the motor
		float distanceLeft = this->myleft->getPosition();
		float distanceRight = this->myright->getPosition();

		//take the average of the two to find a approx distance traveled.
		float increase = (distanceLeft + distanceRight) / 2;

		float angle = IMU->getAngleFromBase();


		float xComponent = increase * cos(angle);
		float yComponent = increase * sin(angle);

		IMU->addToXPosition(xComponent);
		IMU->addToYPosition(yComponent);

		this->driveForward(600 - increase, 200);

		if(isChassisDoneDriving()) {
			this->state = DONE;
		}
	} else if(state == DONE) {

	} else if(state == STANDBY) {

	}

}

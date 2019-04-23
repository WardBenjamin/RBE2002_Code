/*
 * DrivingChassis.h
 *
 *  Created on: Jan 12, 2019
 *      Author: hephaestus
 */

#ifndef DRIVINGCHASSIS_H_
#define DRIVINGCHASSIS_H_
#include "src/pid/PIDMotor.h"
#include "RBEPID.h"
#include "src/commands/GetIMU.h"
#include "config.h"
#include "RangeFinder.h"
#include <math.h>

#define RM_MULT 1
#define LM_MULT -1

#define FORWARD_SPEED 400
#define TURNING_SPEED 400

enum ChassisState {
	DRIVING, DONE, STANDBY
};

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * The 0,0,0 center of the robot is on the ground, half way between the left and right wheel contact points.
 *
 * The +X axis is the positive direction of travel
 *
 * The +Y axis goes from the center of the robot to the left wheel
 *
 * The +Z axis goes from the center up through the robot towards the ceiling.
 *
 * This object should manage the setting of motor setpoints to enable driving
 */
class DrivingChassis {
private:
	PIDMotor * myleft;
	PIDMotor * myright;
	GetIMU * IMU;
	float mywheelTrackMM;
	float mywheelRadiusMM;

	float targetX = 0, targetY = 0, targetTime = 0,
			targetVelocity;
	float startTime, adjustAngle;

	int lastLeftEncoder = 0, lastRightEncoder = 0;

	float lastAngle = 0;

	RBEPID *xPID, *yPID, *anglePID;

	RangeFinder *rf;

	bool isYCorrectionMode, trapzoid_debug = false, joystick_debug = false, showIMU = true, isTurning;

	float targetAngle;

	int steps = 0;

	float localX, localY;

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
	float distanceToWheelAngle(float distance);
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
	float chassisRotationToWheelDistance(float angle);

	void setAngleAdjustment(float angle);
public:
	ChassisState state;

	virtual ~DrivingChassis();

	/**
	 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
	 *
	 * @param left the left motor
	 * @param right the right motor
	 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
	 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
	 */
	DrivingChassis(RangeFinder*, PIDMotor * left, PIDMotor * right, float wheelTrackMM,
			float wheelRadiusMM, GetIMU * imu);

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
	void driveForward(float mmDistanceFromCurrent, int msDuration);
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
	void turnDegrees(float degreesToRotateBase, int msDuration);
	/**
	 * Check to see if the chassis is performing an action
	 *
	 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
	 *
	 *  @note this function is fast-return and should not block
	 */
	bool isChassisDoneDriving();

	void loop();

	float getAngle();
	float getDistanceFromTicks(float);

	float* trapzoid_approx(float, float);

	float* joystick_algorithm(float, float);

	void update(float, float);


	void setTargetX (float x){
		this->targetX = x;
	}
	void setTargetY(float y){
		this->targetY = y;
	}
	void setLastAngle(float angle){
		lastAngle = angle;
	}

	ChassisState getState() const {
		return state;
	}
	GetIMU* getIMU() const {
		return IMU;
	}
	RangeFinder* getRangeFinder() const {
		return rf;
	}

};

#endif /* DRIVINGCHASSIS_H_ */


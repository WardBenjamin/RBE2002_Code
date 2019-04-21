/*
 * RobotGeometry.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef ROBOTGEOMETRY_H_
#define ROBOTGEOMETRY_H_

#include "CityGeometry.h"

#define IN_TO_MM 25.4

#define ROBOT_Y_LENGTH (12 * IN_TO_MM)
#define ROBOT_X_LENGTH (12 * IN_TO_MM)

#define STARTING_ROBOT_Y_OFFSET -(1 * IN_TO_MM)
#define STARTING_ROBOT_X_OFFSET ((13.3 * IN_TO_MM) - ROBOT_Y_LENGTH)

#define STARTING_ROBOT_Y ((ROBOT_Y_LENGTH / 2.0) + STARTING_ROBOT_Y_OFFSET)
#define STARTING_ROBOT_X ((ROBOT_X_LENGTH / 2.0) + STARTING_ROBOT_X_OFFSET)


#define DANGER_MODE_X_LOCAL
#define DANGER_MODE_Y_LOCAL

#endif /* ROBOTGEOMETRY_H_ */

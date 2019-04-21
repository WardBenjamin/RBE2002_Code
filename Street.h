/*
 * Street.h
 *
 *  Created on: Apr 21, 2019
 *      Author: developer
 */

#ifndef STREET_H_
#define STREET_H_

#define DEFAULT_STREET_LENGTH_MM 400
#define DEFAULT_STREET_HEIGHT_MM 200

#define NORTH_INDEX 0
#define EAST_INDEX 1
#define WEST_INDEX 2
#define SOUTH_INDEX 3

class Street {

private:
	char name[12];

	Street *intersects[4];

	float startX, startY, endY, endX, length, height;

public:
	Street(char[12], float, float, float, float);
	virtual ~Street();

	float getGlobalEndpoint();
	float getGlobablStartpoint();

	float getLocalPIDYSetPoint();
	float getLocalPIDXSetPoint();

	float getGlobalPIDYSetPoint();
	float getGlobalPIDXSetPoint();

	float getStreetLength() const { return length; }
	float getStreetHeight() const { return height; }



};

#endif /* STREET_H_ */

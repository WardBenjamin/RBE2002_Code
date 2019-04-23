/*
 * Turrent.h
 *
 *  Created on: Apr 23, 2019
 *      Author: developer
 */

#ifndef TURRENT_H_
#define TURRENT_H_

class Turrent {
public:
	Turrent();
	virtual ~Turrent();

	bool checkFire();
	void fire();

	void sweepNorth();
	void sweepSouth();
	void sweepEast();
	void sweepWest();

};

#endif /* TURRENT_H_ */

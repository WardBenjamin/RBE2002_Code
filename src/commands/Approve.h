

#if !defined(APPROVE_H)
#define APPROVE_H
#include "../ExampleRobot.h"
#include <SimplePacketComs.h>
class Approve: public PacketEventAbstract {
	ExampleRobot* robotPointer;
public:
	// Packet ID needs to be set
	Approve(ExampleRobot* robot) :
			PacketEventAbstract(1994) // Address of this event
	{
		robotPointer = robot;
	}
	//User function to be called when a packet comes in
	// Buffer contains data from the packet coming in at the start of the function
	// User data is written into the buffer to send it back
	void event(float * buffer);
};
#endif

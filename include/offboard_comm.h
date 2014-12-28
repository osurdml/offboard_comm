#include <ros/ros.h>

#include <setpoint_transmitter.h>

class OffboardCommunicator {
public:
	OffboardCommunicator(SetpointTransmitter* transmitter);

	void transmitCallback(const ros::TimerEvent& e);
	
private:
	SetpointTransmitter* transmitter;
};

int main(int argc, char **argv);

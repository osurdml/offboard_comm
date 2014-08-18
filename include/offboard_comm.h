#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <setpoint_transmitter.h>

class OffboardCommunicator {
	public:
	OffboardCommunicator(SetpointTransmitter* transmitter);

	void transmitCallback(const ros::TimerEvent& e);
	
	private:
	boost::shared_ptr<SetpointTransmitter> transmitter;
};

int main(int argc, char **argv);

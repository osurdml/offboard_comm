#include <offboard_comm.h>

OffboardCommunicator::OffboardCommunicator(SetpointTransmitter* transmitter) : transmitter(transmitter) {
}

void OffboardCommunicator::transmitCallback(const ros::TimerEvent& e) {
	transmitter->transmit();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "offboard_comm");
	ros::NodeHandle nh("~");

	SetpointTransmitter transmitter;
	ros::Subscriber velSubscriber = nh.subscribe("/cmd_vel", 10, &SetpointTransmitter::setpointCallback, &transmitter);

	OffboardCommunicator communicator(&transmitter);
	ros::Timer timer = nh.createTimer(ros::Duration(0.10), &OffboardCommunicator::transmitCallback, &communicator);

	ros::spin();

	return 0;
}


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>

#include <boost/shared_ptr.h>

#include <setpoint_transmitter.h>

class OffboardCommunicator {
	public:
	OffboardCommunicator(SetpointTransmitter* transmitter);

	void updateFrontierCallback(const ros::TimerEvent& e);
	void transmitCallback(const ros::TimerEvent& e);
	
	private:
	boost::shared_ptr<actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction>> exploreClient;
	boost::shared_ptr<SetpointTransmitter> transmitter;
};

int main(int argc, char **argv);

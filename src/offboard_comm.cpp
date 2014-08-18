#include <offboard_comm.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

OffboardCommunicator::OffboardCommunicator(SetpointTransmitter* transmitter) {
	this->transmitter = transmitter;
	this->exploreClient = new actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction>("/explore_server", true);

	ROS_INFO("Waiting for frontier_exploration server");
	this->exploreClient->waitForServer();
	ROS_INFO("Connected to frontier_exploration server");
}

void OffboardCommunicator::updateFrontierCallback(const ros::TimerEvent& e) {
	ROS_INFO("Updating frontier");

	geometry_msgs::PolygonStamped boundary;
	boundary.header.frame_id = "/map";

	geometry_msgs::Point32 point1;
	point1.x = -2.0f;
	point1.y = -2.0f;
	point1.z = 0.0f;
	boundary.polygon.points.push_back(point1);

	geometry_msgs::Point32 point2;
	point2.x = -2.0f;
	point2.y = 2.0f;
	point2.z = 0.0f;
	boundary.polygon.points.push_back(point2);

	geometry_msgs::Point32 point3;
	point3.x = 2.0f;
	point3.y = 2.0f;
	point3.z = 0.0f;
	boundary.polygon.points.push_back(point3);

	geometry_msgs::Point32 point4;
	point4.x = 2.0f;
	point4.y = -2.0f;
	point4.z = 0.0f;
	boundary.polygon.points.push_back(point4);

	geometry_msgs::PointStamped center;
	center.header.frame_id = "/map";
	center.point.x = 0.0f;
	center.point.y = 0.0f;
	center.point.z = 0.0f;

	frontier_exploration::ExploreTaskGoal action;
	action.explore_boundary = boundary;
	action.explore_center = center;

	this->exploreClient->sendGoal(action);

	bool finished = this->exploreClient->waitForResult(ros::Duration(5.0));
	if(finished) {
		actionlib::SimpleClientGoalState state = exploreClient->getState();

		ROS_INFO("Got updated frontier");
	} else {
		ROS_WARN("frontier_exploration did not respond in reasonable time");
	}
}

void OffboardCommunicator::transmitCallback(const ros::TimerEvent& e) {
	ROS_INFO("Sending");
	transmitter->transmit();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "offboard_comm");
	ros::NodeHandle nh("~");

	SetpointTransmitter transmitter;
	ros::Subscriber velSubscriber = nh.subscribe("/cmd_vel", 10, &SetpointTransmitter::setpointCallback, &transmitter);

	OffboardCommunicator communicator(&transmitter);
	nh.createTimer(ros::Duration(5.0), &OffboardCommunicator::updateFrontierCallback, &communicator);
	nh.createTimer(ros::Duration(0.10), &OffboardCommunicator::transmitCallback, &communicator);

	ros::spin();

	return 0;
}


#include <offboard_comm.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <frontier_exploration/ExploreTaskAction.h>

#include <setpoint_transmitter.h>

actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> *ac;

void updateFrontierCallback(const ros::TimerEvent& e) {
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

	ac->sendGoal(action);

	bool finished = ac->waitForResult(ros::Duration(5.0));
	if(finished) {
		actionlib::SimpleClientGoalState state = ac->getState();

		ROS_INFO("Got updated frontier");
	} else {
		ROS_WARN("frontier_exploration did not respond in reasonable time");
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "qex_gs");
	ros::NodeHandle nh("~");

	SetpointTransmitter transmitter;
	nh.subscribe("/cmd_vel", 10, &SetpointTransmitter::setpointCallback, &transmitter);

	ac = new actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction>("/explore_server", true);

	ROS_INFO("Waiting for frontier_exploration server");
	ac->waitForServer();
	ROS_INFO("Connected to frontier_exploration server");

	ros::Timer updateFrontierTimer = nh.createTimer(ros::Duration(5.0), updateFrontierCallback);

	ros::Rate rate(10); // 10Hz
	while(ros::ok()) {
		ROS_INFO("Sending...");

		transmitter.transmit();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


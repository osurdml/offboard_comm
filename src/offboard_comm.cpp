#include <offboard_comm.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <frontier_exploration/ExploreTaskAction.h>

#include <setpoint_transmitter.h>

SetpointTransmitter *transmitter;
actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> *ac;


void moveBaseCallback(const move_base_msgs::MoveBaseGoalConstPtr& action) {
  ROS_INFO("Got move_base command");
}

void updateFrontierCallback(const ros::TimerEvent& e) {
	ROS_INFO("Updating frontier");

	geometry_msgs::PolygonStamped boundary;

	geometry_msgs::Point32 point1;
	point1.x = -10.0f;
	point1.y = -10.0f;
	point1.z = 0.0f;
	boundary.polygon.points.push_back(point1);

	geometry_msgs::Point32 point2;
	point2.x = -10.0f;
	point2.y = 10.0f;
	point2.z = 0.0f;
	boundary.polygon.points.push_back(point2);

	geometry_msgs::Point32 point3;
	point3.x = 10.0f;
	point3.y = 10.0f;
	point3.z = 0.0f;
	boundary.polygon.points.push_back(point3);

	geometry_msgs::Point32 point4;
	point4.x = 10.0f;
	point4.y = -10.0f;
	point4.z = 0.0f;
	boundary.polygon.points.push_back(point4);

	geometry_msgs::PointStamped center;
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

	transmitter = new SetpointTransmitter();

	ac = new actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction>("/explore_server", true);

	actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as(nh, "/move_base", &moveBaseCallback, false);
	as.start();

	ROS_INFO("Waiting for frontier_exploration server");
	ac->waitForServer();
	ROS_INFO("Connected to frontier_exploration server");

	ros::Timer updateFrontierTimer = nh.createTimer(ros::Duration(5.0), updateFrontierCallback);

	ros::Rate rate(10); // 10Hz
	while(ros::ok()) {
		ROS_INFO("Sending...");

		transmitter->transmit();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


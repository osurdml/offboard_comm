#include <offboard_comm.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <setpoint_transmitter.h>

SetpointTransmitter *transmitter;

void actionCallback(const move_base_msgs::MoveBaseGoalConstPtr &action) {
	tf::TransformListener listener;
	tf::StampedTransform transform;

	ros::Time now(0);
	listener.waitForTransform("/base_link", "/map", now, ros::Duration(1.0));
	listener.lookupTransform("/base_link", "/map", now, transform);

	transmitter->updateVelocitySetpoint(transform, action->target_pose.pose);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "qex_gs");
	ros::NodeHandle nh("~");

	transmitter = new SetpointTransmitter();

        actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as(nh, "/move_base", &actionCallback, false);
        as.start();

	ros::Rate rate(10); // 10Hz
	while(ros::ok()) {
		// ROS_INFO("Sending...");

		transmitter->transmit();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


#include <offboard_comm.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <setpoint_transmitter.h>

void actionCallback(const move_base_msgs::MoveBaseGoalConstPtr &action) {
  ROS_INFO("GOT ACTION");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "qex_gs");
	ros::NodeHandle nh("~");

	// SetpointTransmitter *transmitter = new SetpointTransmitter();

	// ros::Subscriber qex_gs_tf_proc = nh.subscribe("/tf", 10, &SetpointTransmitter::tfProcCallback, transmitter);
	// ros::Subscriber qex_gs_target_proc = nh.subscribe("/target", 10, &SetpointTransmitter::targetProcCallback, transmitter);

        actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as(nh, "/move_base", &actionCallback, false);
        as.start();

	ros::Rate rate(10); // 5Hz
	while(ros::ok()) {
		// ROS_INFO("Sending...");

		// TODO: Reset SP after some timeout on the TF topic
		// transmitter->transmit();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


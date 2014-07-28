#include <offboard_comm.h>

#include <ros/ros.h>

#include <setpoint_transmitter.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "qex_gs");
	ros::NodeHandle nh("~");

	SetpointTransmitter *transmitter = new SetpointTransmitter();

	ros::Subscriber qex_gs_tf_proc = nh.subscribe("/tf", 10, &SetpointTransmitter::tfProcCallback, transmitter);
	ros::Subscriber qex_gs_target_proc = nh.subscribe("/target", 10, &SetpointTransmitter::targetProcCallback, transmitter);

	ros::Rate rate(5); // 5Hz
	while(ros::ok()) {
		ROS_INFO("Sending...");

		// TODO: Reset SP after some timeout on the TF topic
		transmitter->transmit();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


#include <offboard_comm.h>

#include <boost/asio.hpp>
#include <ros/ros.h>

void update_velocity_sp(int cb_idx, float x, float y, float z)
{
	static float tf_x = 0;
	static float tf_y = 0;
	static float tf_z = 0;
	static float target_x = 0;
	static float target_y = 0;
	static float target_z = 0;

	sp.group = 0;
	sp.mode = MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY;

	if (cb_idx == 0) {
		tf_x = x;
		tf_y = y;
		tf_z = z;
	}
	else if (cb_idx == 1) {
		target_x = x;
		target_y = y;
		target_z = z;
	}

	sp.roll[0]   =  MAX(MIN(target_y - tf_y, 1), -1) * AXIS_SCALE; // vy
	sp.pitch[0]  = -MAX(MIN(target_x - tf_x, 1), -1) * AXIS_SCALE; // vx
	sp.yaw[0]    = 0.0 * AXIS_SCALE; // yawspeed
	sp.thrust[0] = MAX(MIN(0.50 - (target_z - tf_z), 1), -1) * AXIS_SCALE; // vz

	ROS_INFO("Map: %6f %6f %6f   Target: %6f %6f %6f   Setpoint: %6d %6d %6u",
			tf_x, tf_y, tf_z,
			target_x, target_y, target_z,
			sp.roll[0], sp.pitch[0], sp.thrust[0]
			);
}

void tf_proc_callback(const tf::tfMessage &m)
{
	geometry_msgs::TransformStamped f = m.transforms[0];
	if (f.child_frame_id == "/map") {
		//ROS_INFO("Map: %10g %10g %10g\n",
		//		f.transform.translation.x,
		//		f.transform.translation.y,
		//		f.transform.translation.z
		//		);

		update_velocity_sp(0,
				f.transform.translation.x,
				f.transform.translation.y,
				f.transform.translation.z
				);
	}
}

void target_proc_callback(const tf::tfMessage &m)
{
	geometry_msgs::TransformStamped f = m.transforms[0];
	if (f.child_frame_id == "/map") {
		//ROS_INFO("Target: %10g %10g %10g\n",
		//		f.transform.translation.x,
		//		f.transform.translation.y,
		//		f.transform.translation.z
		//		);

		update_velocity_sp(1,
				f.transform.translation.x,
				f.transform.translation.y,
				f.transform.translation.z
				);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "qex_gs");
	ros::NodeHandle nh;

	ros::Subscriber qex_gs_tf_proc = nh.subscribe("/tf", 10, tf_proc_callback);
	ros::Subscriber qex_gs_target_proc = nh.subscribe("/target", 10, target_proc_callback);

	boost::asio::io_service io;
	boost::asio::serial_port serial(io, "/dev/ttyUSB0");
	serial.set_option(boost::asio::serial_port_base::baud_rate(57600));
	serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial.set_option(boost::asio::serial_port::character_size(8));

	char buf[BUFFER_SIZE];

	ros::Rate rate(50); // 50Hz
	while(ros::ok() && serial.is_open()) {
		mavlink_message_t message;

		mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(255, 0, &message, &sp);

		unsigned len = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
		boost::asio::write(serial, boost::asio::buffer(buf, len));

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


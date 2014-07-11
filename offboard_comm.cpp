#include <offboard_comm.h>

#include <boost/asio.hpp>
#include <mavlink.h>
#include <ros/ros.h>

void tf_proc_callback(const tf::tfMessage &m)
{
	geometry_msgs::TransformStamped f = m.transforms[0];
	if (f.child_frame_id == "/map") {
		ROS_INFO("%10g %10g %10g\n",
				f.transform.rotation.x,
				f.transform.rotation.y,
				f.transform.rotation.z
				);

	}
	// TODO: Calculate desired velocity vector from tf and setpoint
	// TODO(yoos): Do something useful with the tf info
}

void sp_proc_callback(const tf::tfMessage &m)
{
	geometry_msgs::TransformStamped f = m.transforms[0];
	if (f.child_frame_id == "/map") {
		ROS_INFO("Setpoint: %10g %10g %10g\n",
				f.transform.rotation.x,
				f.transform.rotation.y,
				f.transform.rotation.z
				);

	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "qex_gs");
	ros::NodeHandle nh;

	ros::Subscriber qex_gs_tf_proc = nh.subscribe("/tf", 10, tf_proc_callback);
	ros::Subscriber qex_gs_sp_proc = nh.subscribe("/sp", 10, sp_proc_callback);

	boost::asio::io_service io;
	boost::asio::serial_port serial(io, "/dev/ttyUSB0");
	serial.set_option(boost::asio::serial_port_base::baud_rate(57600));
	serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial.set_option(boost::asio::serial_port::character_size(8));

	char buf[BUFFER_SIZE];

	ros::Rate rate(50); // 50Hz
	while(ros::ok() && serial.is_open()) {
		ROS_INFO("Sending...");

		mavlink_message_t message;
		mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t sp;

		sp.group = 0;
		sp.mode = MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY;

		sp.roll[0] = 0.0 * AXIS_SCALE; // vy
		sp.pitch[0] = 0.0 * AXIS_SCALE; // vx
		sp.yaw[0] = 0.0 * AXIS_SCALE; // yawspeed
		sp.thrust[0] = 0.50 * AXIS_SCALE; // vz

		mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(255, 0, &message, &sp);

		unsigned len = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
		boost::asio::write(serial, boost::asio::buffer(buf, len));

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


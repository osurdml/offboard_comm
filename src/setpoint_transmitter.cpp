#include <setpoint_transmitter.h>

#include <string.h>

SetpointTransmitter::SetpointTransmitter() {
	serial_port = serial_port_ptr(new boost::asio::serial_port(io, "/dev/ttyUSB0"));
	serial_port->set_option(boost::asio::serial_port_base::baud_rate(57600));
	serial_port->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial_port->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial_port->set_option(boost::asio::serial_port::character_size(8));

	memset(&sp, 0, sizeof(sp));

	sp.group = 0;
	sp.mode = MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY;
}

SetpointTransmitter::~SetpointTransmitter() {
}

void SetpointTransmitter::setpointCallback(const geometry_msgs::Twist& twist) {
	sp.roll[0]   =  (int16_t)  (limit(-1.0, 1.0, twist.linear.y) * AXIS_SCALE); // vy
	sp.pitch[0]  =  (int16_t)  (limit(-1.0, 1.0, twist.linear.x) * AXIS_SCALE); // vx
	sp.yaw[0]    =  (int16_t)  (limit(-1.0, 1.0, twist.angular.z) * AXIS_SCALE); // yawspeed
	// NOTE: PX4/Firmware modified to accept positional thrust values rather
	// than velocities.
	sp.thrust[0] =  (uint16_t) (1.5 * AXIS_SCALE); // z

	ROS_INFO("Setpoint: %6d %6d %6d %6u",
			sp.roll[0], sp.pitch[0], sp.yaw[0], sp.thrust[0]
		);
}

void SetpointTransmitter::transmit() {
	mavlink_message_t message;

	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(0xFF, 0, &message, &sp);

	uint8_t buf[BUFFER_SIZE];
	unsigned len = mavlink_msg_to_send_buffer(buf, &message);

	boost::asio::write(*serial_port, boost::asio::buffer(buf, len));
}

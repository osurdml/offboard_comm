#include <setpoint_transmitter.h>

#include <cstring>

SetpointTransmitter::SetpointTransmitter() : serial_port(io, "/dev/ttyACM0") {
	serial_port.set_option(boost::asio::serial_port_base::baud_rate(57600));
	serial_port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial_port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial_port.set_option(boost::asio::serial_port::character_size(8));

	std::memset(&sp, 0, sizeof(sp));

	// set all bits but 4, 5, 6 (vx, vy, vz), and 12 (yaw_rate)
	sp.type_mask = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 11);
	sp.type_mask = ~sp.type_mask;

	// TODO: only MAV_FRAME_LOCAL_NED is supported in PX4?
	sp.coordinate_frame = MAV_FRAME_BODY_NED;
}

SetpointTransmitter::~SetpointTransmitter() {
}

void SetpointTransmitter::setpointCallback(const geometry_msgs::Twist& twist) {
	// TODO: everything is in m/s now, still need AXIS_SCALE?
	// TODO: need negative signs?
	sp.vx = -(limit(-1.0, 1.0, twist.linear.x / 10.0) * AXIS_SCALE);
	sp.vy = -(limit(-1.0, 1.0, twist.linear.y / 10.0) * AXIS_SCALE);
	sp.yaw_rate = -(limit(-1.0, 1.0, twist.angular.z / 10.0) * AXIS_SCALE);
	sp.z = -1.50;

	ROS_INFO("Setpoint: vx=%6f vy=%6f yaw=%6f z=%6f",
			sp.vx, sp.vz, sp.yaw_rate, sp.z
		);
}

void SetpointTransmitter::transmit() {
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(0xFF, 0, &message, &sp);

	uint8_t buf[MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN];
	unsigned len = mavlink_msg_to_send_buffer(buf, &message);

	boost::asio::write(serial_port, boost::asio::buffer(buf, len));
}

#include <setpoint_transmitter.h>

SetpointTransmitter::SetpointTransmitter() {
	serial_port = serial_port_ptr(new boost::asio::serial_port(io, "/dev/ttyUSB0"));
}

SetpointTransmitter::~SetpointTransmitter() {
}

void SetpointTransmitter::tfProcCallback(const tf::tfMessage& m) {
	geometry_msgs::TransformStamped f = m.transforms[0];
	if (m.transforms[0].child_frame_id == "/map") {
		tf = m.transforms[0].transform;
	}
	updateVelocitySetpoint();
}

void SetpointTransmitter::targetProcCallback(const tf::tfMessage& m) {
	geometry_msgs::TransformStamped f = m.transforms[0];
	if (m.transforms[0].child_frame_id == "/map") {
		target = m.transforms[0].transform;
	}
	updateVelocitySetpoint();
}

void SetpointTransmitter::updateVelocitySetpoint() {
	sp.group = 0;
	sp.mode = MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY;

	sp.roll[0]   =  limit(-1.0, 1.0, target.translation.y - tf.translation.y) * AXIS_SCALE; // vy
	sp.pitch[0]  = -limit(-1.0, 1.0, target.translation.x - tf.translation.x) * AXIS_SCALE; // vx
	sp.yaw[0]    =  0.0 * AXIS_SCALE; // yawspeed
	sp.thrust[0] =  limit(-1.0, 1.0, 0.50 - (target.translation.z - tf.translation.z)) * AXIS_SCALE; // vz

	ROS_INFO("Map: %6f %6f %6f   Target: %6f %6f %6f   Setpoint: %6d %6d %6u",
			tf.translation.x, tf.translation.y, tf.translation.z,
			target.translation.x, target.translation.y, target.translation.z,
			sp.roll[0], sp.pitch[0], sp.thrust[0]
			);
}

void SetpointTransmitter::transmit() {
	mavlink_message_t message;

	mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(255, 0, &message, &sp);

	uint8_t buf[BUFFER_SIZE];
	unsigned len = mavlink_msg_to_send_buffer(buf, &message);

	boost::asio::write(*serial_port, boost::asio::buffer(buf, len));
}

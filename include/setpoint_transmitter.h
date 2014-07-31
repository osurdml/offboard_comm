#ifndef SETPOINT_TRANSMITTER_H
#define SETPOINT_TRANSMITTER_H

#include <algorithm>

#include <boost/asio.hpp>
#include <mavlink.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

#define BUFFER_SIZE 300

#define AXIS_SCALE 1000.0f

#define MAVLINK_OFFBOARD_CONTROL_MODE_NONE 0
#define MAVLINK_OFFBOARD_CONTROL_MODE_RATES 1
#define MAVLINK_OFFBOARD_CONTROL_MODE_ATTITUDE 2
#define MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY 3
#define MAVLINK_OFFBOARD_CONTROL_MODE_POSITION 4

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class SetpointTransmitter {
public:
	SetpointTransmitter();
	~SetpointTransmitter();

	void updateVelocitySetpoint(tf::Transform tf, geometry_msgs::Pose goal);
	void transmit();

private:
	double limit(double min, double max, double v) {
		return std::max(std::min(v, max), min);
	}

	mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t sp;

	boost::asio::io_service io;
	serial_port_ptr serial_port;
};

#endif

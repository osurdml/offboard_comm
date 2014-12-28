#ifndef SETPOINT_TRANSMITTER_H
#define SETPOINT_TRANSMITTER_H

#include <algorithm>

#include <boost/asio.hpp>

#include <mavlink.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define AXIS_SCALE 1000.0f

class SetpointTransmitter {
public:
	SetpointTransmitter();
	~SetpointTransmitter();

	void setpointCallback(const geometry_msgs::Twist& twist);
	void transmit();

private:
	double limit(double min, double max, double v) {
		return std::max(std::min(v, max), min);
	}

	mavlink_set_position_target_local_ned_t sp;

	boost::asio::io_service io;
	boost::asio::serial_port serial_port;
};

#endif

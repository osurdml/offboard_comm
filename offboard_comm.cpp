#include <iostream>
#include <fstream>

#include <boost/asio.hpp>
#include <mavlink.h>

#define BUFFER_SIZE 300

#define AXIS_SCALE 1000.0f

#define MAVLINK_OFFBOARD_CONTROL_MODE_NONE 0
#define MAVLINK_OFFBOARD_CONTROL_MODE_RATES 1
#define MAVLINK_OFFBOARD_CONTROL_MODE_ATTITUDE 2
#define MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY 3
#define MAVLINK_OFFBOARD_CONTROL_MODE_POSITION 4

int main(void) {
	boost::asio::io_service io;
	boost::asio::serial_port serial(io, "/dev/ttyUSB0");
	serial.set_option(boost::asio::serial_port_base::baud_rate(57600));
	serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	serial.set_option(boost::asio::serial_port::character_size(8));

	char buf[BUFFER_SIZE];
	while(serial.is_open()) {
		std::cout << "Sending..." << std::endl;

		mavlink_message_t message;
		mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t sp;

		sp.group = 0;
		sp.mode = MAVLINK_OFFBOARD_CONTROL_MODE_VELOCITY;

		sp.roll[0] = 0.0 * AXIS_SCALE;
		sp.pitch[0] = 0.0 * AXIS_SCALE;
		sp.yaw[0] = 0.0 * AXIS_SCALE;
		sp.thrust[0] = 0.50 * AXIS_SCALE;

		mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(255, 0, &message, &sp);
		unsigned len = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
		boost::asio::write(serial, boost::asio::buffer(buf, len));

		// Sleep 20ms for ~50Hz communication
		usleep(2e4);
	}

	std::cerr << "Serial connection closed" << std::endl;

	return 0;
}


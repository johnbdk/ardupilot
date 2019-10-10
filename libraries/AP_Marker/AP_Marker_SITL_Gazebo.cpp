#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Marker_SITL_Gazebo.h"
#include <SITL/SITL.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>

extern const AP_HAL::HAL& hal;

AP_Marker_SITL_Gazebo::AP_Marker_SITL_Gazebo(): 
	_last_timestamp(0),
	sock(true)
{}

void AP_Marker_SITL_Gazebo::init(int8_t bus)
{
	SITL::SITL *sitl = AP::sitl();
	// try to bind to a specific port so that if we restart ArduPilot
  	// Gazebo keeps sending us packets. Not strictly necessary but
  	// useful for debugging
	sock.bind("127.0.0.1", sitl->marker_port);

	sock.reuseaddress();
	sock.set_blocking(false);

	hal.console->printf("AP_Marker_SITL_Gazebo::init()\n");
	printf("AP_Marker_SITL_Gazebo::init()\n");
	_flags.healthy = true;
}

// retrive latest sensor data - returns true if new data is available
bool AP_Marker_SITL_Gazebo::update()
{
	// return immediately if not healthy
	if (!_flags.healthy) {
		return false;
	}

	// receive packet from a ROS Node

	/*
	 reply packet sent from simulator to ArduPilot
	*/
	struct marker_packet {
		uint32_t timestamp; 	// in miliseconds
		uint32_t num_targets;
		float pos_x;
		float pos_y;
		float pos_z;
		float distance;
		float size_x;
		float size_y;
	} pkt;
	const int wait_ms = 0;
	ssize_t s = sock.recv(&pkt, sizeof(marker_packet), wait_ms);

	bool new_data = false;

	// printf("size: %lu, pkt.time %u, last_time %u\n", sizeof(marker_packet), pkt.timestamp, _last_timestamp);
	if (s == sizeof(marker_packet) && pkt.timestamp > _last_timestamp) {
		// fprintf(stderr, "posx %f posy %f posz %f sizex %f sizey %f\n", pkt.pos_x, pkt.pos_y, pkt.pos_z, pkt.size_x, pkt.size_y);
		_target_info.timestamp = pkt.timestamp;
		_target_info.pos_x = pkt.pos_x;
		_target_info.pos_y = pkt.pos_y;
		_target_info.pos_z = pkt.pos_z;
		_target_info.distance = pkt.distance;
		_last_timestamp = pkt.timestamp;
		_last_update_ms = _last_timestamp;
		new_data = true;
	}

	// return true if new data found
	return new_data;
}
#endif // CONFIG_HAL_BOARD

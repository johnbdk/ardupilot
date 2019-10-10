#pragma once

#include <AP_HAL/utility/Socket.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "Marker.h"

class AP_Marker_SITL_Gazebo : public Marker
{
public:
	AP_Marker_SITL_Gazebo();

	// init - initialize sensor library
	void init(int8_t bus) override;

	// retrive latest sensor data - returns true if new data is available
	bool update() override;

private:
	uint32_t _last_timestamp;
	SocketAPM sock;
};
#endif // CONFIG_HAL_BOARD

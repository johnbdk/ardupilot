#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "Marker.h"
#include <SITL/SITL.h>

class AP_Marker_SITL : public Marker
{
public:
	// init - initialize sensor library
	void init(int8_t bus) override;

	// retrive latest sensor data = returns true if new data is available
	bool update() override;

private:
	SITL::SITL *_sitl; // sitl instance pointer
	uint32_t _last_timestamp = 0;
};
#endif // CONFIG_HAL_BOARD

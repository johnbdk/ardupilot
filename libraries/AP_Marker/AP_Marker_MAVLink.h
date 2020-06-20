/*
 * AP_Marker_MAVLink.h
 *
 */
#pragma once

#include "Marker.h"
#include <AP_HAL/AP_HAL.h>

class AP_Marker_MAVLink : public Marker
{
public:
    // init - initialize sensor library
    void init(int8_t bus) override;

    // retrieve latest sensor data - returns true if new data is available
    bool update() override;

    // Get update from mavlink
    void handle_msg(const mavlink_message_t &msg);

    void log_marker();

private:
	uint32_t _timestamp_us;	// timestamp from message
};

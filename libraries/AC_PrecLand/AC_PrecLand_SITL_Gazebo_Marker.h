#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_Marker/AP_Marker_SITL_Gazebo.h>
#include <stdint.h>
#include <iostream>

/*
 * AC_PrecLand_SITL_Gazebo_Marker - implements precision landing using target
 * vectors provided Gazebo via a network socket
 */

class AC_PrecLand_SITL_Gazebo_Marker : public AC_PrecLand_Backend
{
public:
	// Constructor
	AC_PrecLand_SITL_Gazebo_Marker(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

	// perform any required initialisation of backend
	void init() override;

	// retrieve updates from sensor
	void update() override;

	// provides a unit vector towards the target in body frame
	// returns same as have_los_meas()
	bool get_los_body(Vector3f& ret) override;

	// returns system time in milliseconds of last los measurement
	uint32_t los_meas_time_ms() override;

	// return true if there is a valid los measurement available
	bool have_los_meas() override;

	// returns distance to target in meters (0 means distance is not known)
    float distance_to_target() override;

    // parses a mavlink fault injection message from the companion computer
    void handle_fault_injection_msg(const mavlink_message_t &msg) override;
    
    void print_sensor_state(uint32_t last_update_ms);

private:
	AP_Marker_SITL_Gazebo marker;

	float               _distance_to_target;    // distance from the camera to target in meters
	Vector3f			_los_meas_body;			// unit vector in body frame pointing towards target
	bool				_have_los_meas;			// true if there is a valid measurement from the camera
	uint32_t			_los_meas_time_ms;		// system time in milliseconds when los was measured

	float fi_rate_marker, fi_rate_marker_rem, fi_error_marker;
    bool took_fi_marker, apply_error_marker;
};

#endif

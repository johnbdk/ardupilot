#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_SITL_Gazebo_Marker.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

// Constructor
AC_PrecLand_SITL_Gazebo_Marker::AC_PrecLand_SITL_Gazebo_Marker(const AC_PrecLand &frontend, AC_PrecLand::precland_state &state)
	: AC_PrecLand_Backend(frontend, state),
		marker()
{
}

// init - perform initialisation of this backend
void AC_PrecLand_SITL_Gazebo_Marker::init()
{
	marker.init(get_bus());
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_SITL_Gazebo_Marker::update()
{
	// update health
	_state.healthy = marker.healthy();

	// get new sensor data
	marker.update();

	if (marker.num_targets() > 0 && marker.last_update_ms() != _los_meas_time_ms) {
		// _have_los_meas = marker.get_unit_vector_body(_los_meas_body);
		marker.get_unit_vector_body(_los_meas_body);
		marker.get_distance_to_target(_distance_to_target);
		_have_los_meas = true;
		_los_meas_time_ms = marker.last_update_ms();
        printf("HAL %u, los_time %u, Time %d, IS? %d\n", AP_HAL::millis(), _los_meas_time_ms, AP_HAL::millis()-_los_meas_time_ms, AP_HAL::millis()-_los_meas_time_ms <= 1000);
	}
	_have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

// provides a unit vector towards the target in body frame
// returns same as have_los_meas()
bool AC_PrecLand_SITL_Gazebo_Marker::get_los_body(Vector3f& ret) {
	if (have_los_meas()) {
		ret = _los_meas_body;
		return true;
	}
	return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_SITL_Gazebo_Marker::los_meas_time_ms() {
	return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_SITL_Gazebo_Marker::have_los_meas() {
	return _have_los_meas;
}

// return distance to target
float AC_PrecLand_SITL_Gazebo_Marker::distance_to_target()
{
    return _distance_to_target;
}

#endif

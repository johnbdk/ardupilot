#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_SITL_Gazebo_Fusion.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

// Constructor
AC_PrecLand_SITL_Gazebo_Fusion::AC_PrecLand_SITL_Gazebo_Fusion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
	: AC_PrecLand_Backend(frontend, state),
	  irlock(),
	  marker(),
	  _have_los_meas_irlock(false),
	  _have_los_meas_marker(false)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_SITL_Gazebo_Fusion::init()
{
	irlock.init(get_bus());
	marker.init(get_bus());
}

// update - give a chance to driver to get updates from sensor
void AC_PrecLand_SITL_Gazebo_Fusion::update()
{
	// update health
	_state.healthy = irlock.healthy() | marker.healthy();

	// get new sensor data
	irlock.update();
	marker.update();

	if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas_time_ms_irlock) {
		irlock.get_unit_vector_body(_los_meas_body_irlock);
		_have_los_meas_irlock = true;
		_los_meas_time_ms_irlock = irlock.last_update_ms();
		printf("1: hal_time %u, los_time %u, diff %d, |diff| %d, healthy %d\n",
            AP_HAL::millis(),
            _los_meas_time_ms_irlock,
            AP_HAL::millis()-_los_meas_time_ms_irlock,
            static_cast<int>(AP_HAL::millis()-_los_meas_time_ms_irlock),
            static_cast<int>(AP_HAL::millis()-_los_meas_time_ms_irlock) <= 1000);
	}
	_have_los_meas_irlock = _have_los_meas_irlock && static_cast<int>(AP_HAL::millis()-_los_meas_time_ms_irlock) <= 1000;

	if (marker.num_targets() > 0 && marker.last_update_ms() != _los_meas_time_ms_marker) {
		marker.get_unit_vector_body(_los_meas_body_marker);
		marker.get_distance_to_target(_distance_to_target);
		_have_los_meas_marker = true;
		_los_meas_time_ms_marker = marker.last_update_ms();
		printf("2: hal_time %u, los_time %u, diff %d, |diff| %d, healthy %d\n",
            AP_HAL::millis(),
            _los_meas_time_ms_marker,
            AP_HAL::millis()-_los_meas_time_ms_marker,
            static_cast<int>(AP_HAL::millis()-_los_meas_time_ms_marker),
            static_cast<int>(AP_HAL::millis()-_los_meas_time_ms_marker) <= 1000);
	}
	_have_los_meas_marker = _have_los_meas_marker && static_cast<int>(AP_HAL::millis()-_los_meas_time_ms_marker) <= 1000;

	_los_meas_time_ms = (_los_meas_time_ms_marker > _los_meas_time_ms_irlock) ? _los_meas_time_ms_marker : _los_meas_time_ms_irlock;
    _have_los_meas = (_have_los_meas_marker | _have_los_meas_irlock) && static_cast<int>(AP_HAL::millis()-_los_meas_time_ms) <= 1000;
	fuse_vectors();

}

// provides a unit vector towards the target in body frame
// returns same as have_los_meas();
bool AC_PrecLand_SITL_Gazebo_Fusion::get_los_body(Vector3f& ret) {
	if (have_los_meas()) {
		ret = _los_meas_body;
		return true;
	}
	return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_SITL_Gazebo_Fusion::los_meas_time_ms() {
	return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_SITL_Gazebo_Fusion::have_los_meas() {
	return _have_los_meas;
}

bool AC_PrecLand_SITL_Gazebo_Fusion::fuse_vectors() {

	if (_have_los_meas_marker && _have_los_meas_irlock) {
		_los_meas_body.x = (_los_meas_body_irlock.x + _los_meas_body_marker.x) / 2;
		_los_meas_body.y = (_los_meas_body_irlock.y + _los_meas_body_marker.y) / 2;
		_los_meas_body.z = _los_meas_body_marker.z;
	}
	else if (_have_los_meas_marker && !_have_los_meas_irlock) {
		_los_meas_body.x = _los_meas_body_marker.x;
		_los_meas_body.y = _los_meas_body_marker.y;
		_los_meas_body.z = _los_meas_body_marker.z;
	}
	else {
		_los_meas_body.x = _los_meas_body_irlock.x;
		_los_meas_body.y = _los_meas_body_irlock.y;
		_los_meas_body.z = _los_meas_body_irlock.z;
	}
	return true;
}

int8_t AC_PrecLand_SITL_Gazebo_Fusion::which_sensor() {

	if (_have_los_meas_marker && _have_los_meas_irlock) {
		return 0;
	}
	else if (_have_los_meas_marker && !_have_los_meas_irlock) {
		return 1;
	}
	else {
		return 2;
	}
}

// return distance to target
float AC_PrecLand_SITL_Gazebo_Fusion::distance_to_target()
{
    return _distance_to_target;
}

#endif

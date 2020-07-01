#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_Fusion.h"

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_Fusion::AC_PrecLand_Fusion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
	: AC_PrecLand_Backend(frontend, state),
	  irlock(),
	  marker(),
	  _have_los_meas(false),
	  _have_los_meas_irlock(false),
	  _have_los_meas_marker(false),
	  _los_meas_time_ms(0),
	  _los_meas_time_ms_irlock(0),
	  _los_meas_time_ms_marker(0),
	  real_marker_fusion_count(0),
	  real_irlock_fusion_count(0),
	  fi_rate_marker(0.0f),
	  fi_rate_irlock(0.0f),
	  fi_rate_marker_rem(0.0f),
	  fi_rate_irlock_rem(0.0f),
	  fi_error_marker(0.0f),
	  fi_error_irlock(0.0f),
	  took_fi_marker(false),
	  took_fi_irlock(false),
	  apply_error_marker(false),
	  apply_error_irlock(false)
{
	// Initialize random number generator 
    srand((unsigned) AP_HAL::millis());
}

// init - perform initialisation of this backend
void AC_PrecLand_Fusion::init()
{
	irlock.init(get_bus());
	marker.init(get_bus());
}

// Process VISUAL_MARKER_TARGET mavlink message
void AC_PrecLand_Fusion::handle_msg(const mavlink_message_t &msg)
{
    marker.handle_msg(msg);
}

// Process SET_FAULT_INJECTION mavlink message
void AC_PrecLand_Fusion::handle_fault_injection_msg(const mavlink_message_t &msg)
{   
	__mavlink_set_fault_injection_t packet;
	mavlink_msg_set_fault_injection_decode(&msg, &packet);
	
	switch (packet.target_subsystem) {
		case 0:
			fi_rate_marker  = packet.fi_rate_marker;
			fi_error_marker = packet.fi_error_marker;
			took_fi_marker  = true;
			fi_rate_irlock  = packet.fi_rate_irlock;
			fi_error_irlock = packet.fi_error_irlock;
			took_fi_irlock  = true;
			break;
		case 1:
			fi_rate_marker  = packet.fi_rate_marker;
			fi_error_marker = packet.fi_error_marker;
			took_fi_marker  = true;
			break;
		case 2:
			fi_rate_irlock  = packet.fi_rate_irlock;
			fi_error_irlock = packet.fi_error_irlock;
			took_fi_irlock  = true;
			break;
	}
}

// update - give a chance to driver to get updates from sensor
void AC_PrecLand_Fusion::update()
{
	// update health
	_state.healthy = irlock.healthy() | marker.healthy();

	// IRLOCK FAULT INJECTION
	if (is_positive(fi_rate_irlock)) {
		if (took_fi_irlock == false) {
			fi_rate_irlock_rem = is_zero(fi_rate_irlock_rem) ? fi_rate_irlock : fi_rate_irlock_rem;
		}
		else if (took_fi_irlock == true) {
			fi_rate_irlock_rem = fi_rate_irlock;
			took_fi_irlock = false;
		}
	}
	else if (is_negative(fi_rate_irlock)) {
		fi_rate_irlock_rem = (rand() % 100) < fabsf(fi_rate_irlock);
	}
	else { //is_zero(fi_rate_irlock)
		fi_rate_irlock_rem = 0.0f;
	}

	// MARKER FAULT INJECTION
	if (is_positive(fi_rate_marker)) {
		if (took_fi_marker == false) {
			fi_rate_marker_rem = is_zero(fi_rate_marker_rem) ? fi_rate_marker : fi_rate_marker_rem;
		}
		else if (took_fi_marker == true) {
			fi_rate_marker_rem = fi_rate_marker;
			took_fi_marker = false;
		}
	}
	else if (is_negative(fi_rate_marker)) {
		fi_rate_marker_rem = (rand() % 100) < fabsf(fi_rate_marker);
	}
	else { //is_zero(fi_rate_marker)
		fi_rate_marker_rem = 0.0f;
	}

	irlock_update();
	marker_update();
	_los_meas_time_ms = (marker.last_update_ms() > irlock.last_update_ms()) ? marker.last_update_ms() : irlock.last_update_ms();
    _have_los_meas = _have_los_meas_marker | _have_los_meas_irlock;
    fuse_vectors();
}

void AC_PrecLand_Fusion::irlock_update()
{
	if (is_negative(fi_rate_irlock)) {
		if (is_zero(fi_rate_irlock_rem)) {
			// get new sensor data
			irlock.update();
		}
		// FINALLY AN INJECTION FAULT
		else { //!is_zero(fi_rate_irlock_rem)
			if (!is_zero(fi_error_irlock)) {
				irlock.update();
				apply_error_irlock = true;
			}
			else { //is_zero(fi_error_irlock)
				// avoid to update the irlock sensor values --> SIMPLY DROP
			}
		}
	}
	else { // (is_positive(fi_rate_irlock) || is_zero(fi_rate_irlock))
		if (!is_equal(fi_rate_irlock_rem, fi_rate_irlock) || is_zero(fi_rate_irlock)) {
			// get new sensor data
			irlock.update();
		}
		// FINALLY AN INJECTION FAULT
		else { // is_equal(fi_rate_irlock_rem, fi_rate_irlock)
			if (!is_zero(fi_error_irlock)) {
				irlock.update();
				apply_error_irlock = true;
			}
			else { //is_zero(fi_error_irlock)
				// avoid to update the irlock sensor values --> SIMPLY DROP
			}
		}

		if (is_positive(fi_rate_irlock_rem)) {
			fi_rate_irlock_rem--;
		}
	}

	if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas_time_ms_irlock) {
		irlock.get_unit_vector_body(_los_meas_body_irlock);
		if (apply_error_irlock == true) {
			_los_meas_body_irlock.x += _los_meas_body_irlock.x*(fi_error_irlock/100.0f);
			_los_meas_body_irlock.y += _los_meas_body_irlock.y*(fi_error_irlock/100.0f);
		}
		_have_los_meas_irlock = true;
	}
	_have_los_meas_irlock = _have_los_meas_irlock && AP_HAL::millis()-irlock.last_update_ms() <= 1000;
	apply_error_irlock = false;
}

void AC_PrecLand_Fusion::marker_update()
{
	if (is_negative(fi_rate_marker)) {
		if (is_zero(fi_rate_marker_rem)) {
			// get new sensor data
			marker.update();
		}
		// FINALLY AN INJECTION FAULT
		else { //!is_zero(fi_rate_marker_rem)
			if (!is_zero(fi_error_marker)) {
				marker.update();
				apply_error_marker = true;
			}
			else { //is_zero(fi_error_marker)
				// avoid to update the marker sensor values --> SIMPLY DROP
			}
		}
	}
	else { // (is_positive(fi_rate_marker) || is_zero(fi_rate_marker))
		if (!is_equal(fi_rate_marker_rem, fi_rate_marker) || is_zero(fi_rate_marker)) {
			// get new sensor data
			marker.update();
		}
		// FINALLY AN INJECTION FAULT
		else { // is_equal(fi_rate_marker_rem, fi_rate_marker)
			if (!is_zero(fi_error_marker)) {
				marker.update();
				apply_error_marker = true;
			}
			else { //is_zero(fi_error_marker)
				// avoid to update the marker sensor values --> SIMPLY DROP
			}
		}

		if (is_positive(fi_rate_marker_rem)) {
			fi_rate_marker_rem--;
		}
	}
	
	if (marker.num_targets() > 0 && marker.last_update_ms() != _los_meas_time_ms_marker) {
		marker.get_distance_to_target(_distance_to_target);
		marker.get_unit_vector_body(_los_meas_body_marker);
		if (apply_error_marker == true) {
			_los_meas_body_marker.x += _los_meas_body_marker.x*(fi_error_marker/100.0f);
			_los_meas_body_marker.y += _los_meas_body_marker.y*(fi_error_marker/100.0f);
			// _los_meas_body_marker.z += _los_meas_body_marker.z*(fi_error_marker/100.0f);
		}
		_have_los_meas_marker = true;
	}
	_have_los_meas_marker = _have_los_meas_marker && AP_HAL::millis()-marker.last_update_ms() <= 1000;
	apply_error_marker = false;
}

bool AC_PrecLand_Fusion::fuse_vectors()
{
	if ((_have_los_meas_marker && _have_los_meas_irlock)) {
		if ((marker.last_update_ms() != _los_meas_time_ms_marker) && (irlock.last_update_ms() != _los_meas_time_ms_irlock)) {
			set_fusion_los_meas_body();
			_los_meas_time_ms_marker = marker.last_update_ms();
			_los_meas_time_ms_irlock = irlock.last_update_ms();
			real_marker_fusion_count = 1;
			real_irlock_fusion_count = 1;
		}
		else if ((marker.last_update_ms() == _los_meas_time_ms_marker) && (irlock.last_update_ms() != _los_meas_time_ms_irlock)) {
			// if not did it twice
			if (real_irlock_fusion_count < 2) {
				set_fusion_los_meas_body();
				real_irlock_fusion_count++;
				real_marker_fusion_count = 1;
			}
			else {
				set_irlock_los_meas_body();
				real_marker_fusion_count = 0;
			}
			_los_meas_time_ms_irlock = irlock.last_update_ms();
		}
		else if ((marker.last_update_ms() != _los_meas_time_ms_marker) && (irlock.last_update_ms() == _los_meas_time_ms_irlock)) {
			// if not did it once, the bellow is just the complecity, we need only real_marker_fusion_count < 1 for now
			// TODO check the 2nd case of the below if statement
			if (real_marker_fusion_count < 1 || ((marker.last_update_ms() - irlock.last_update_ms()) < 50)) {
				set_fusion_los_meas_body();
				real_marker_fusion_count++;
				if (marker.last_update_ms() - irlock.last_update_ms() < 50) {
					real_irlock_fusion_count = 0;
				}
				else {
					real_irlock_fusion_count = 1;
				}
			}
			else {
				set_marker_los_meas_body();
				real_irlock_fusion_count = 0;
			}
			_los_meas_time_ms_marker = marker.last_update_ms();
		}
		sensor_alive = 0;
	}
	else if ((_have_los_meas_marker && !_have_los_meas_irlock) && (marker.last_update_ms() != _los_meas_time_ms_marker)) {
		set_marker_los_meas_body();
		_los_meas_time_ms_marker = marker.last_update_ms();
		sensor_alive = 1;
		real_marker_fusion_count = 0;
		real_irlock_fusion_count = 0;
	}
	else if ((!_have_los_meas_marker && _have_los_meas_irlock) && (irlock.last_update_ms() != _los_meas_time_ms_irlock)) {
		set_irlock_los_meas_body();
		_los_meas_time_ms_irlock = irlock.last_update_ms();
		sensor_alive = 2;
		real_marker_fusion_count = 0;
		real_irlock_fusion_count = 0;
	}
	else {
		return false;
	}
	return true;
}

void AC_PrecLand_Fusion::set_marker_los_meas_body()
{
	_los_meas_body.x = _los_meas_body_marker.x;
	_los_meas_body.y = _los_meas_body_marker.y;
	_los_meas_body.z = _los_meas_body_marker.z;
}

void AC_PrecLand_Fusion::set_irlock_los_meas_body()
{
	_los_meas_body.x = _los_meas_body_irlock.x;
	_los_meas_body.y = _los_meas_body_irlock.y;
	_los_meas_body.z = _los_meas_body_irlock.z;
}

void AC_PrecLand_Fusion::set_fusion_los_meas_body()
{
	_los_meas_body.x = (_los_meas_body_irlock.x + _los_meas_body_marker.x) / 2;
	_los_meas_body.y = (_los_meas_body_irlock.y + _los_meas_body_marker.y) / 2;
	_los_meas_body.z = _los_meas_body_marker.z;
}

int8_t AC_PrecLand_Fusion::which_sensor()
{
	return sensor_alive;
}

// return distance to target
float AC_PrecLand_Fusion::distance_to_target()
{
    return _distance_to_target;
}

// provides a unit vector towards the target in body frame
// returns same as have_los_meas();
bool AC_PrecLand_Fusion::get_los_body(Vector3f& ret)
{
	if (have_los_meas()) {
		ret = _los_meas_body;
		return true;
	}
	return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Fusion::los_meas_time_ms()
{
	return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_Fusion::have_los_meas()
{
	return _have_los_meas;
}

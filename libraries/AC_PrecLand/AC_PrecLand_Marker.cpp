#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_Marker.h"

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_Marker::AC_PrecLand_Marker(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      marker(),
      _los_meas_time_ms(0),
      _distance_to_target(0.0f),
      fi_rate_marker(0.0f),
      fi_rate_marker_rem(0.0f),
      fi_error_marker(0.0f),
      took_fi_marker(false),
      apply_error_marker(false)
{
    // Initialize random number generator 
    srand((unsigned) AP_HAL::millis());
}

// init - perform initialisation of this backend
void AC_PrecLand_Marker::init()
{
    marker.init(get_bus());
}

void AC_PrecLand_Marker::handle_msg(const mavlink_message_t &msg)
{
    marker.handle_msg(msg);
}

// Process SET_FAULT_INJECTION mavlink message
void AC_PrecLand_Marker::handle_fault_injection_msg(const mavlink_message_t &msg)
{   
    __mavlink_set_fault_injection_t packet;
    mavlink_msg_set_fault_injection_decode(&msg, &packet);
    
    if (packet.target_subsystem == 1) {
        fi_rate_marker  = packet.fi_rate_marker;
        fi_error_marker = packet.fi_error_marker;
        took_fi_marker  = true;
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_Marker::update()
{
    // update health
    _state.healthy = marker.healthy();

    // MARKER FAULT INJECTION
    // CHOOSE VALUES
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
    
    // APPLY FAULT INJECTION IF NECESSARY
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
    
    if (marker.num_targets() > 0 && marker.last_update_ms() != _los_meas_time_ms) {
        marker.get_distance_to_target(_distance_to_target);
        marker.get_unit_vector_body(_los_meas_body);
        if (apply_error_marker == true) {
            _los_meas_body.x += _los_meas_body.x*(fi_error_marker/100.0f);
            _los_meas_body.y += _los_meas_body.y*(fi_error_marker/100.0f);
            _los_meas_body.z += _los_meas_body.z*(fi_error_marker/100.0f);
        }
        _have_los_meas = true;
        _los_meas_time_ms = marker.last_update_ms();
    }
    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
    apply_error_marker = false;
}

// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_Marker::get_los_body(Vector3f& ret) {
    if (have_los_meas()) {
        ret = _los_meas_body;
        return true;
    }
    return false;
}

// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_Marker::get_los_body_log(Vector3f& ret) {
    if (have_los_meas()) {
        marker.get_vector_body(ret);
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Marker::los_meas_time_ms() {
    return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_Marker::have_los_meas() {
    return _have_los_meas;
}

// return distance to target
float AC_PrecLand_Marker::distance_to_target()
{
    return _distance_to_target;
}

int8_t AC_PrecLand_Marker::which_sensor()
{
    return 1;
}

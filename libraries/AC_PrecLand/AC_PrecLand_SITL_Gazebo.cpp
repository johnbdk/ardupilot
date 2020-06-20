#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_SITL_Gazebo.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

// Constructor
AC_PrecLand_SITL_Gazebo::AC_PrecLand_SITL_Gazebo(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock(),
      fi_rate_irlock(0.0f),
      fi_rate_irlock_rem(0.0f),
      fi_error_irlock(0.0f),
      took_fi_irlock(false),
      apply_error_irlock(false)
      // _los_meas_time_ms(0)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_SITL_Gazebo::init()
{
    irlock.init(get_bus());
}

// Process SET_FAULT_INJECTION mavlink message
void AC_PrecLand_SITL_Gazebo::handle_fault_injection_msg(const mavlink_message_t &msg)
{   
    __mavlink_set_fault_injection_t packet;
    mavlink_msg_set_fault_injection_decode(&msg, &packet);
    
    if (packet.target_subsystem == 2) {
        fi_rate_irlock  = packet.fi_rate_irlock;
        fi_error_irlock = packet.fi_error_irlock;
        took_fi_irlock  = true;
        printf("TOOK MAVLINK MASSAGE SET FAULT INJECTION, IRLOCK\n");
        printf("FI_R_I: %f, FI_E_I %f\n", fi_rate_irlock, fi_error_irlock);
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_SITL_Gazebo::update()
{
    // update health
    _state.healthy = irlock.healthy();

    // IRLOCK FAULT INJECTION
    // CHOOSE VALUES
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

    // APPLY FAULT INJECTION IF NECESSARY
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
                // printf("IRLOCK SIMPLY DROP\n");
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
                // printf("IRLOCK SIMPLY DROP\n");
            }
        }

        if (is_positive(fi_rate_irlock_rem)) {
            fi_rate_irlock_rem--;
        }
    }

    if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas_time_ms) {
        irlock.get_unit_vector_body(_los_meas_body);
        if (apply_error_irlock == true) {
            // printf("IRLOCK DROP WITH ERROR %f\n", fi_error_irlock/100.0f);
            // printf("IRLOCK prev x:%f, y:%f, z:%f\n", _los_meas_body.x, _los_meas_body.y, _los_meas_body.z);
            _los_meas_body.x += _los_meas_body.x*(fi_error_irlock/100.0f);
            _los_meas_body.y += _los_meas_body.y*(fi_error_irlock/100.0f);
            // printf("IRLOCK after x:%f, y:%f, z:%f\n", _los_meas_body.x, _los_meas_body.y, _los_meas_body.z);
        }
        _have_los_meas = true;
        _los_meas_time_ms = irlock.last_update_ms();
        //print_sensor_state(_los_meas_time_ms);
    }
    _have_los_meas = _have_los_meas && static_cast<int>(AP_HAL::millis()-_los_meas_time_ms) <= 1000;
    apply_error_irlock = false;
}

// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_SITL_Gazebo::get_los_body(Vector3f& ret)
{
    if (have_los_meas()) {
        ret = _los_meas_body;
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_SITL_Gazebo::los_meas_time_ms()
{
    return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_SITL_Gazebo::have_los_meas()
{
    return _have_los_meas;
}

void AC_PrecLand_SITL_Gazebo::print_sensor_state(uint32_t last_update_ms) {
    printf("I: hal_time %u, los_time %u, diff %d, healthy %d\n",
        AP_HAL::millis(),
        last_update_ms,
        static_cast<int>(AP_HAL::millis() - last_update_ms),
        static_cast<int>(AP_HAL::millis() - last_update_ms) <= 1000);
}

#endif

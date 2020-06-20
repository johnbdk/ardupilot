#pragma once

#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_IRLock/AP_IRLock.h>
#include <AP_Marker/AP_Marker.h>

/*
 * AC_PrecLand_Fusion - implements precision landing using fusion
 * target vectors provided Gazebo via a network socket. Fusion is consist of
 * an IRLock and a Visual Marker
 */

class AC_PrecLand_Fusion : public AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_Fusion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // perform any required initialisation of backend
    void init() override;

    // calls update methods from both sensor (irlock and marker)
    void update() override;

    // retrieve update from irlock
    void irlock_update();

    // retrieve update from marker
    void marker_update();

    // sets _los_meas_body vectror from the marker sensor
    void set_marker_los_meas_body();

    // sets _los_meas_body vectror from the irlock sensor
    void set_irlock_los_meas_body();

    // sets _los_meas_body vectror from both (marker & irlock) sensors
    void set_fusion_los_meas_body();

    // provides a unit vector towards the targer in body frame
    // returns same as have_los_meas()
    bool get_los_body(Vector3f& ret) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas() override;

    // return true if the fusion of irlock and marker vectors were successful
    bool fuse_vectors();

    int8_t which_sensor() override;

    // returns distance to target in meters (0 means distance is not known)
    float distance_to_target() override;

    // parses a mavlink message from the companion computer
    void handle_msg(const mavlink_message_t &msg) override;

    // parses a mavlink fault injection message from the companion computer
    void handle_fault_injection_msg(const mavlink_message_t &msg) override;

private:
    AP_IRLock_I2C       irlock;
    AP_Marker_MAVLink   marker;

    float       _distance_to_target;        // distance from the camera to target in meters
    Vector3f    _los_meas_body_irlock;
    Vector3f    _los_meas_body_marker;
    Vector3f    _los_meas_body;             // unit vector in body frame pointing towards target
    bool        _have_los_meas_irlock;  
    bool        _have_los_meas_marker;
    bool        _have_los_meas;             // true if there is a valid measurement from the camera
    uint32_t    _los_meas_time_ms;          // system time in milliseconds when los was measured
    uint32_t    _los_meas_time_ms_irlock;
    uint32_t    _los_meas_time_ms_marker;
    uint8_t     sensor_alive;               // 0: both marker & irlock, 1: marker, 2: irlock
    int         real_marker_fusion_count;
    int         real_irlock_fusion_count;

    float fi_rate_marker, fi_rate_marker_rem, fi_error_marker;
    float fi_rate_irlock, fi_rate_irlock_rem, fi_error_irlock;
    bool took_fi_irlock, apply_error_irlock;
    bool took_fi_marker, apply_error_marker;
};

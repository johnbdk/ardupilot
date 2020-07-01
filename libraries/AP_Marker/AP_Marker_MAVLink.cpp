#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_Marker_MAVLink.h"
#include <stdio.h>
#include <utility>
#include <AP_Logger/AP_Logger.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <iostream>
#endif

extern const AP_HAL::HAL& hal;

// AP_Marker_MAVLink::AP_Marker_MAVLink() {}

void AP_Marker_MAVLink::init(int8_t bus)
{
    _flags.healthy = true;
    _last_update_ms = 0;
    // AP::logger().Write("MRKS", "TimeUS,OfsX,OfsY,OfsZ,Dist", "Iffff", AP_HAL::millis(), (double)_target_info.pos_x, (double)_target_info.pos_y, (double)_target_info.pos_z, (double)_target_info.distance);
    // printf("Starting Marker on MAVLink\n");
}

/*
   Set the pose estimation based on a MAVLINK message
*/
void AP_Marker_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    __mavlink_visual_marker_target_t packet;
    mavlink_msg_visual_marker_target_decode(&msg, &packet);
    // printf("TOOK MAVLINK MASSAGE MARKER\n");
    _timestamp_us = packet.time_usec;

    _target_info.timestamp = AP_HAL::millis();
    // printf("%u\n", _target_info.timestamp);
    _target_info.pos_x = packet.x;
    _target_info.pos_y = packet.y;
    _target_info.pos_z = packet.z;
    // _target_info.distance = sqrtf(_target_info.pos_x*_target_info.pos_x + _target_info.pos_y*_target_info.pos_y + _target_info.pos_z*_target_info.pos_z);
    _target_info.distance = norm(_target_info.pos_x, _target_info.pos_y, _target_info.pos_z);
    // printf("pos_x:%f, pos_y:%f, pos_z:%f, dist:%f\n", _target_info.pos_x, _target_info.pos_y, _target_info.pos_z, _target_info.distance);
    //AP::logger().Write("MRKS", "TimeUS,OfsX,OfsY,OfsZ,Dist", "Iffff", AP_HAL::millis(), (double)_target_info.pos_x, (double)_target_info.pos_y, (double)_target_info.pos_z, (double)_target_info.distance);
    log_marker();
}

// retrieve latest sensor data - returns true if new data is available
bool AP_Marker_MAVLink::update()
{
    bool new_data = false;

    if (_last_update_ms != _target_info.timestamp) {
        new_data = true;
    }
    _last_update_ms = _target_info.timestamp;
    // _flags.healthy = (AP_HAL::millis() - _last_read_ms < 100);

    // return true if new data found
    return new_data;
}

void AP_Marker_MAVLink::log_marker()
{
    struct log_marker pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MARKER_MSG),
        time_us : _target_info.timestamp,
        posx    : -_target_info.pos_y*100,
        posy    : _target_info.pos_x*100,
        posz    : _target_info.pos_z*100,
        dist    : _target_info.distance*100
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

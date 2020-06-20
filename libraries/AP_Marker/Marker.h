#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <unistd.h>
#include <iostream>

class Marker
{
public:
	// init - initialize sensor library
	// library won't be usable unless this is first called
	virtual void init(int8_t bus) = 0;

	// true if marker sensor is online and healthy
	bool healthy() const { return _last_update_ms; }

	// timestamp of most recent data read from the sensor
	uint32_t last_update_ms() const { return _last_update_ms; }
	
	// returns the number of markers in the current frame
	size_t num_targets() const { return _flags.healthy?1:0; }

	// retrieve latest sensor data - returns true if new data is available	
	virtual bool update() = 0;

	// retrieve body frame unit vector in direction of target
	// returns true if data is available
	bool get_unit_vector_body(Vector3f& ret) const;

	bool get_vector_body(Vector3f& ret) const;

	bool get_distance_to_target(float& dist) const;

protected:
	struct AP_Marker_Flags {
		uint8_t healthy : 1; // true if sensor is healthy
	} _flags;

	// internals
	uint32_t _last_update_ms;

	// marker_target_info
	typedef struct {
		uint32_t timestamp;	// milliseconds since system start
		float pos_x;		// x-axis distance from center of image to center of target in units of tan(theta)
		float pos_y;		// y-axis distance from center of image to center of target in units of tan(theta)
		float pos_z;
		float distance;
	} marker_target_info;

	marker_target_info _target_info;
};

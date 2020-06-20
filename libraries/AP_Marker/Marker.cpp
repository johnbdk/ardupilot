#include "Marker.h"

// retrieve body frame unit vector in direction of target
// returns true if data is availabe
bool Marker::get_unit_vector_body(Vector3f& ret) const
{
	// return false if we have no target
	if (!_flags.healthy) {
		return false;
	}

	// use data from lowest marker's number
	ret.x = -_target_info.pos_y;
	ret.y = _target_info.pos_x;
	ret.z = _target_info.pos_z;
	// printf("1: x:%f, y:%f, z:%f\n", ret.x, ret.y, ret.z);
	if (!is_zero(ret.length())) { 
		ret /= ret.length();
	}
	// printf("2: x:%f, y:%f, z:%f\n", ret.x, ret.y, ret.z);
	return true;
}

// retrieve body frame unit vector in direction of target
// returns true if data is availabe
bool Marker::get_vector_body(Vector3f& ret) const
{
	// return false if we have no target
	if (!_flags.healthy) {
		return false;
	}

	// use data from lowest marker's number
	ret.x = -_target_info.pos_y;
	ret.y = _target_info.pos_x;
	ret.z = _target_info.pos_z;
	// printf("2: x:%f, y:%f, z:%f\n", ret.x, ret.y, ret.z);
	return true;
}

bool Marker::get_distance_to_target(float& dist) const
{
	// return false if we have no target
	if (!_flags.healthy) {
		return false;
	}

	dist = _target_info.distance;
	return true;
}

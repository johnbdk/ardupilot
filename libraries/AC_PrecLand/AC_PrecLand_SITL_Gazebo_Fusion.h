#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_IRLock/AP_IRLock_SITL_Gazebo.h>
#include <AP_Marker/AP_Marker_SITL_Gazebo.h>
#include <unistd.h>
#include <iostream>

/*
 * AC_PrecLand_SITL_Gazebo_Fusion - implements precision landing using fusion
 * target vectors provided Gazebo via a network socket. Fusion is consist of
 * an IRLock and a Visual Marker
 */

class AC_PrecLand_SITL_Gazebo_Fusion : public AC_PrecLand_Backend
{
public:

	// Constructor
	AC_PrecLand_SITL_Gazebo_Fusion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

	// perform any required initialisation of backend
	void init() override;

	// retrieve updates from sensor
	void update() override;

	// provides a unit vector towards the targer in body frame
	// returns same as have_los_meas()
	bool get_los_body(Vector3f& ret) override;

	// returns system time in milliseconds of last los measurement
	uint32_t los_meas_time_ms() override;

	// return true if there is a valid los measurement available
	bool have_los_meas() override;

	// return true if the fusion of irlock and marker vectors were successful
	bool fuse_vectors();

	int8_t which_sensor();

	// returns distance to target in meters (0 means distance is not known)
    float distance_to_target() override;

private:
	AP_IRLock_SITL_Gazebo irlock;
	AP_Marker_SITL_Gazebo marker;

	float		_distance_to_target;    	// distance from the camera to target in meters
	Vector3f	_los_meas_body_irlock;
	Vector3f	_los_meas_body_marker;
	Vector3f	_los_meas_body; 			// unit vector in body frame pointing towards target
	bool		_have_los_meas_irlock; 	
	bool		_have_los_meas_marker;
	bool		_have_los_meas; 			// true if there is a valid measurement from the camera
	uint32_t	_los_meas_time_ms;			// system time in milliseconds when los was measured
	uint32_t	_los_meas_time_ms_irlock;
	uint32_t	_los_meas_time_ms_marker;
};

#endif

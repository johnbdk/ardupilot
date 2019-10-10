#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Marker_SITL.h"
#include "AP_AHRS/AP_AHRS.h"

void AP_Marker_SITL::init(int8_t bus)
{
	_sitl = AP::sitl();
	_sitl->precland_sim._type.set_and_notify(SITL::SIM_Precland::PreclandType::PRECLAND_TYPE_CONE);
}

// retrieve latest sensor data - returns true if new data is available
bool AP_Marker_SITL::update()
{
	//returns immiediately if not healthy
	_flags.healthy = _sitl->precland_sim.healthy();
	if (!_flags.healthy) {
		return false;
	}

	if (_sitl->precland_sim.last_update_ms() != _last_timestamp) {
		const Vector3f position = _sitl->precland_sim.get_target_position();
		const Matrix3f &body_to_ned = AP::ahrs().get_rotation_body_to_ned();
		const Vector3f real_position = body_to_ned.mul_transpose(-position);
		_last_timestamp = _sitl->precland_sim.last_update_ms();
		_last_update_ms = _last_timestamp;
		_target_info.timestamp = _last_timestamp;
		_target_info.pos_x = real_position.y;
		_target_info.pos_y = -real_position.x;
		_target_info.pos_z = real_position.z;
		return true;
	}
	return false;
}
#endif // CONFIG_HAL_BOARD

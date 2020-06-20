#include "Marker.h"
#include "AP_Marker_MAVLink.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Marker_SITL_Gazebo.h"
#include "AP_Marker_SITL.h"
#endif

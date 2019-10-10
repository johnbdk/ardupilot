#include "Marker.h"
#include "AP_Marker_I2C.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Marker_SITL_Gazebo.h"
#include "AP_Marker_SITL.h"
#endif

#ifndef ROBOT_DEFINE_H_
#define ROBOT_DEFINE_H_

#include "device_define.h"

/* Robot physical parameters ============================================== */
#define ROBOT_PI 3.1415926f
#define WHEEL_RADIUS 0.076f
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.6221f

#define WHEEL_RPM_TO_WHEEL_MPS \
    (2.0f * ROBOT_PI * WHEEL_RADIUS / 60.0f / M3508_REDUCTION_RATIO)
#define WHEEL_MPS_TO_WHEEL_RPM (1.0f / WHEEL_RPM_TO_WHEEL_MPS)
#define ROBOT_RPS_TO_WHEEL_MPS (2.0f * ROBOT_PI * ROBOT_CENTER_TO_WHEEL_RADIUS)
#define WHEEL_MPS_TO_ROBOT_RPS (1.0f / ROBOT_RPS_TO_WHEEL_MPS)

#endif

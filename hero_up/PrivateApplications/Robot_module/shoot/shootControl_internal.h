#ifndef SHOOT_CONTROL_INTERNAL_H_
#define SHOOT_CONTROL_INTERNAL_H_

#include "shootControl.h"

#include "general_config_label.h"

#define SHOOT_FRIC_DEFAULT_SPEED_RPM 3110.0f
#define SHOOT_FRIC_FRONT_SPEED_RPM 4300.0f
#define SHOOT_FRIC_BACK_SPEED_RPM 4320.0f
#define SHOOT_FRIC_IDLE_SPEED_RPM 200.0f
#define SHOOT_FRIC_PID_KP 17.5f
#define SHOOT_FRIC_PID_KI 0.0f
#define SHOOT_FRIC_PID_KD 12.5f
#define SHOOT_FRIC_PID_INTEGRAL_LIMIT 0.0f
typedef struct {
    ShootControl control;
} shoot_runtime_t;

#define fric_target_speed_rpm (g_shoot_runtime.control.ShootTargetInput.fric_speed_rpm)
#define fric_speed_pid (g_shoot_runtime.control.ShootMotorControl.fric_speed_pid)
#define fric_target_output (g_shoot_runtime.control.ShootMotorControl.fric_target_output)

#endif

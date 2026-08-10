#ifndef WORLD_GIMBAL_INTERNAL_H_
#define WORLD_GIMBAL_INTERNAL_H_

#include "worldGimbal.h"

#define WORLD_GIMBAL_IK_MAX_ITERS 10U
#define WORLD_GIMBAL_IK_LAMBDA 0.01f
#define WORLD_GIMBAL_IK_MAX_STEP_RAD 0.05f
#define WORLD_GIMBAL_IK_CONVERGE_RAD 0.0005f
#define WORLD_GIMBAL_DEG_TO_RAD 0.01745329252f
#define WORLD_GIMBAL_RAD_TO_DEG 57.2957795131f
#define WORLD_GIMBAL_HALF_TURN_D 180.0f
#define WORLD_GIMBAL_FULL_ROTATION_D 360.0f
#define WORLD_GIMBAL_UNIT_MIN (-1.0f)
#define WORLD_GIMBAL_UNIT_MAX 1.0f
#define WORLD_GIMBAL_NORMALIZE_EPSILON 1.0e-9f
#define WORLD_GIMBAL_HORIZONTAL_EPSILON 0.001f
#define WORLD_GIMBAL_INPUT_EPSILON_D 1.0e-6f
#define WORLD_GIMBAL_SINGULAR_EPSILON 1.0e-12f

#define world_target_vector (g_world_gimbal.WorldGimbalTargetInput.f_des_B)
#define world_last_right_vector (g_world_gimbal.WorldGimbalTargetInput.last_right_B)
#define world_last_right_valid (g_world_gimbal.WorldGimbalTargetInput.last_right_valid)
#define world_target_initialized (g_world_gimbal.WorldGimbalTargetInput.init_done)
#define world_gravity_vector (g_world_gimbal.WorldGimbalEstimate.g_B)
#define world_chassis_roll_d (g_world_gimbal.WorldGimbalEstimate.chassis_roll_deg)
#define world_chassis_pitch_d (g_world_gimbal.WorldGimbalEstimate.chassis_pitch_deg)
#define world_chassis_yaw_d (g_world_gimbal.WorldGimbalEstimate.chassis_yaw_deg)
#define world_real_vector (g_world_gimbal.WorldGimbalEstimate.f_real_B)
#define world_pitch_axis_vector (g_world_gimbal.WorldGimbalEstimate.b_B)
#define world_angle_error_d (g_world_gimbal.WorldGimbalEstimate.angle_error_deg)
#define world_yaw_d (g_world_gimbal.WorldGimbalEstimate.world_yaw_deg)
#define world_pitch_d (g_world_gimbal.WorldGimbalEstimate.world_pitch_deg)
#define world_yaw_command_d (g_world_gimbal.WorldGimbalControl.q_yaw_cmd_deg)
#define world_pitch_command_d (g_world_gimbal.WorldGimbalControl.q_pitch_cmd_deg)
#define world_yaw_command_rad (g_world_gimbal.WorldGimbalControl.q_yaw_cmd_rad)
#define world_pitch_command_rad (g_world_gimbal.WorldGimbalControl.q_pitch_cmd_rad)
#define world_ik_converged (g_world_gimbal.WorldGimbalControl.converged)
#define world_ik_iterations (g_world_gimbal.WorldGimbalControl.iters_used)
#define world_gimbal_enabled (g_world_gimbal.enable)

#endif

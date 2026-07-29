#ifndef GIMBAL_CONTROL_INTERNAL_H_
#define GIMBAL_CONTROL_INTERNAL_H_

#include "gimbalControl.h"

#include "algorism.h"
#include "general_define.h"

#define GIMBAL_MOUSE_FILTER_ALPHA 0.7f
#define GIMBAL_SNIPER_YAW_GAIN_D 0.1f
#define GIMBAL_SNIPER_PITCH_GAIN_D 0.03f
#define GIMBAL_NORMAL_YAW_GAIN_D 1.0f
#define GIMBAL_NORMAL_PITCH_GAIN_D (-0.5f)
#define GIMBAL_REMOTE_PITCH_SCALE 2.0f
#define GIMBAL_SEPARATE_INPUT_SCALE 1.0f
#define GIMBAL_KEY_HOLD_THRESHOLD_CYCLES 10U
#define GIMBAL_KEY_HOLD_STEP_D 0.01f
#define GIMBAL_KEY_TAP_STEP_D 0.1f
#define GIMBAL_MOUSE_GAIN_NORMAL 0.02f
#define GIMBAL_MOUSE_GAIN_SNIPER 0.015f
#define GIMBAL_HALF_TURN_D 180.0f
#define GIMBAL_POSE_WARMUP_CYCLES 200U
#define GIMBAL_TARGET_GUARD_CYCLES 30U
#define GIMBAL_KEY_RELEASED 0
#define GIMBAL_PITCH_FIXED_DISABLED 0U
#define GIMBAL_PITCH_FIXED_ENABLED 1U

typedef struct {
    GimbalControl control;
    SmoothFilter mouse_filter_x;
    SmoothFilter mouse_filter_y;
    float yaw_trim_d;
    float pitch_trim_d;
    uint32_t yaw_key_hold_cycles;
    uint32_t pitch_key_hold_cycles;
    int8_t last_yaw_key_direction;
    int8_t last_pitch_key_direction;
    uint8_t target_align_delay_cycles;
    uint8_t pitch_fixed_mode;
    uint8_t previous_sniper_mode;
    uint8_t last_ctrl_terminal;
} gimbal_runtime_t;

#define gimbal_rc_fixed_pitch rc_ch1
#define gimbal_rc_yaw rc_ch2
#define gimbal_rc_pitch rc_ch3
#define gimbal_yaw_key_direction key_ad_direction
#define gimbal_pitch_key_direction key_ws_direction
#define gimbal_mouse_speed_x rc_mouse_speed_x
#define gimbal_mouse_speed_y rc_mouse_speed_y

#define gimbal_target_yaw_d (g_gimbal_runtime.control.GimbalTargetInput.yaw_angle_d)
#define gimbal_target_pitch_d (g_gimbal_runtime.control.GimbalTargetInput.pitch_angle_d)
#define gimbal_yaw_d (g_gimbal_runtime.control.GimbalEstimate.yaw_angle_d)
#define gimbal_yaw_dps (g_gimbal_runtime.control.GimbalEstimate.yaw_angular_velocity_dps)
#define gimbal_pitch_d (g_gimbal_runtime.control.GimbalEstimate.pitch_angle_d)
#define gimbal_pitch_dps (g_gimbal_runtime.control.GimbalEstimate.pitch_angular_velocity_dps)
#define gimbal_roll_d (g_gimbal_runtime.control.GimbalEstimate.roll_angle_d)
#define gimbal_roll_dps (g_gimbal_runtime.control.GimbalEstimate.roll_angular_velocity_dps)
#define mouse_filter_x (g_gimbal_runtime.mouse_filter_x)
#define mouse_filter_y (g_gimbal_runtime.mouse_filter_y)
#define yaw_trim_d (g_gimbal_runtime.yaw_trim_d)
#define pitch_trim_d (g_gimbal_runtime.pitch_trim_d)
#define yaw_key_hold_cycles (g_gimbal_runtime.yaw_key_hold_cycles)
#define pitch_key_hold_cycles (g_gimbal_runtime.pitch_key_hold_cycles)
#define last_yaw_key_direction (g_gimbal_runtime.last_yaw_key_direction)
#define last_pitch_key_direction (g_gimbal_runtime.last_pitch_key_direction)
#define target_align_delay_cycles (g_gimbal_runtime.target_align_delay_cycles)
#define pitch_fixed_mode (g_gimbal_runtime.pitch_fixed_mode)
#define previous_sniper_mode (g_gimbal_runtime.previous_sniper_mode)
#define last_ctrl_terminal (g_gimbal_runtime.last_ctrl_terminal)

#endif

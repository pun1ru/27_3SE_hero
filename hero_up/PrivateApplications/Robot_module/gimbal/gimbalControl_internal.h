#ifndef GIMBAL_CONTROL_INTERNAL_H_
#define GIMBAL_CONTROL_INTERNAL_H_

#include "gimbalControl.h"
#include "general_config_label.h"

#define GIMBAL_PITCH_UPPER_LIMIT_D 41.5f
#define GIMBAL_PITCH_LOWER_LIMIT_D (-7.0f)
#define GIMBAL_HALF_TURN_D 180.0f
#define GIMBAL_FULL_ROTATION_D 360.0f
#define GIMBAL_RAD_TO_DEG 57.29578f
#define GIMBAL_DEG_TO_RAD 0.01745329252f
#define GIMBAL_POSE_WARMUP_CYCLES 200U
#define GIMBAL_OUTPUT_GUARD_CYCLES 30U
#define GIMBAL_PITCH_INITIAL_TARGET_D 4.9f
#define GIMBAL_PITCH_OUTPUT_LIMIT 800.0f
#define GIMBAL_PITCH_ENCODER_SCALE 1000.0f
#define GIMBAL_PITCH_ENCODER_MIN (-9000.0f)
#define GIMBAL_PITCH_ENCODER_MAX 30000.0f
#define GIMBAL_PITCH_ENCODER_ZERO 244360U
#define GIMBAL_SNIPER_MAX_SPEED_DPS 50U
#define GIMBAL_SPIN_CLOCKWISE 0x00U
#define GIMBAL_SPIN_COUNTERCLOCKWISE 0x01U
#define GIMBAL_YAW_KF_REALIGN_REQUIRED 1U
#define GIMBAL_YAW_KF_ALIGNED 0U

#define GIMBAL_PITCH_LTD_R 20.0f
#define GIMBAL_PITCH_LTD_H 0.003f
#define GIMBAL_PITCH_LTD_MIN_D (-30.0f)
#define GIMBAL_PITCH_LTD_MAX_D 60.0f
#define GIMBAL_PITCH_LTD_KI 0.02f
#define GIMBAL_PITCH_LTD_ERROR_SUM_MAX 90.0f
#define GIMBAL_PITCH_LTD_FILTER 0.3f
#define GIMBAL_PITCH_CONTROL_PERIOD_S (CONTROL_TASK_PERIOD_SET / 1000.0f)

#define GIMBAL_PITCH_ESO_WO 20.0f
#define GIMBAL_ESO_SECOND_ORDER_COEFFICIENT 3.0f
#define GIMBAL_PITCH_ESO_INPUT_GAIN 3.0f
#define GIMBAL_PITCH_ESO_Z3_GAIN 0.30f
#define GIMBAL_PITCH_ESO_Z3_LIMIT 2000.0f
#define GIMBAL_PITCH_ESF_KP 70.0f
#define GIMBAL_PITCH_ESF_KD 8.0f
#define GIMBAL_PITCH_ESF_ERROR_LIMIT 50.0f
#define GIMBAL_PITCH_ESF_OUTPUT_LIMIT 1000.0f
#define GIMBAL_YAW_ESF_WC 6.0f
#define GIMBAL_ESF_DAMPING_COEFFICIENT 2.0f
#define GIMBAL_YAW_ESO_WO 22.0f
#define GIMBAL_YAW_TD_R 20.0f
#define GIMBAL_YAW_CONTROL_PERIOD_S 0.002f
#define GIMBAL_YAW_TD_GAIN 1.0f
#define GIMBAL_YAW_ESO_INPUT_GAIN 40.0f
#define GIMBAL_YAW_ESO_Z3_LIMIT 10000.0f
#define GIMBAL_YAW_ESF_OUTPUT_LIMIT 50000.0f
#define GIMBAL_YAW_MIN_RAD (-3.14159f)
#define GIMBAL_YAW_MAX_RAD 3.14159f
#define GIMBAL_YAW_LTD_R 20.0f
#define GIMBAL_YAW_POSITION_KP 8.5f
#define GIMBAL_YAW_POSITION_KI 0.15f
#define GIMBAL_YAW_POSITION_OUTPUT_LIMIT 4000.0f
#define GIMBAL_YAW_POSITION_INTEGRAL_LIMIT 800.0f
#define GIMBAL_YAW_SPEED_KP 0.09f
#define GIMBAL_YAW_SPEED_KD 0.01f
#define GIMBAL_YAW_SPEED_OUTPUT_LIMIT 10.0f
#define GIMBAL_YAW_KF_PROCESS_NOISE 0.0000001f
#define GIMBAL_YAW_KF_MEASUREMENT_NOISE 0.0000188825f
#define GIMBAL_YAW_DM_FORWARD_OFFSET_RAD (-2.54f)

typedef struct {
    GimbalControl control;
    ScalarKalmanFilter yaw_encoder_kalman_filter;
    float pitch_angle_from_encoder_d;
    uint8_t pose_warmup_cycles;
    uint8_t yaw_kf_realign;
    uint8_t previous_sniper_mode;
} gimbal_runtime_t;

#define gimbal_target_pitch_d (g_gimbal_runtime.control.GimbalTargetInput.pitch_angle_d)
#define gimbal_target_yaw_d (g_gimbal_runtime.control.GimbalTargetInput.yaw_angle_d)
#define gimbal_target_yaw_dps (g_gimbal_runtime.control.GimbalTargetInput.yaw_angular_velocity_dps)
#define gimbal_pitch_d (g_gimbal_runtime.control.GimbalEstimate.pitch_angle_d)
#define gimbal_pitch_dps (g_gimbal_runtime.control.GimbalEstimate.pitch_angular_velocity_dps)
#define gimbal_yaw_d (g_gimbal_runtime.control.GimbalEstimate.yaw_angle_d)
#define gimbal_yaw_dps (g_gimbal_runtime.control.GimbalEstimate.yaw_angular_velocity_dps)
#define gimbal_roll_d (g_gimbal_runtime.control.GimbalEstimate.roll_angle_d)
#define gimbal_roll_dps (g_gimbal_runtime.control.GimbalEstimate.roll_angular_velocity_dps)
#define pitch_controller (g_gimbal_runtime.control.GimbalMotorControl.pitch_angle_adrc)
#define pitch_ltd (g_gimbal_runtime.control.GimbalMotorControl.pitch_LTD)
#define pitch_target_output (g_gimbal_runtime.control.GimbalMotorControl.pitch_target_output)
#define yaw_controller (g_gimbal_runtime.control.GimbalMotorControl.yaw_ADRC)
#define yaw_ltd (g_gimbal_runtime.control.GimbalMotorControl.yaw_LTD)
#define yaw_position_pid (g_gimbal_runtime.control.GimbalMotorControl.yaw_pos_pid)
#define yaw_speed_pid (g_gimbal_runtime.control.GimbalMotorControl.yaw_speed_pid)
#define yaw_target_output (g_gimbal_runtime.control.GimbalMotorControl.yaw_target_output)
#define sniper_position (g_gimbal_runtime.control.GimbalMotorControl.sniper_pos)
#define sniper_max_speed_dps (g_gimbal_runtime.control.GimbalMotorControl.sniper_max_speed)
#define sniper_spin_direction (g_gimbal_runtime.control.GimbalMotorControl.spin_dir)
#define yaw_encoder_kalman_filter (g_gimbal_runtime.yaw_encoder_kalman_filter)
#define pitch_angle_from_encoder_d (g_gimbal_runtime.pitch_angle_from_encoder_d)
#define pose_warmup_cycles (g_gimbal_runtime.pose_warmup_cycles)
#define yaw_kf_realign (g_gimbal_runtime.yaw_kf_realign)
#define previous_sniper_mode (g_gimbal_runtime.previous_sniper_mode)

#endif

#ifndef CHASSIS_CONTROL_INTERNAL_H_
#define CHASSIS_CONTROL_INTERNAL_H_

#include "chassisControl.h"

#include "algorism.h"
#include "general_define.h"

#define CHASSIS_WHEEL_COUNT 4U
#define CHASSIS_ZERO_OUTPUT 0
#define CHASSIS_INPUT_INACTIVE 0U
#define CHASSIS_INPUT_ACTIVE 1U

#define CHASSIS_FOLLOW_PID_KP (-0.02f)
#define CHASSIS_FOLLOW_PID_KI 0.0f
#define CHASSIS_FOLLOW_PID_KD 0.008f
#define CHASSIS_FOLLOW_PID_INTEGRAL_LIMIT 0.0f
#define CHASSIS_FOLLOW_PID_OUTPUT_LIMIT 3.0f
#define CHASSIS_SPEED_COMPENSATE_PID_KP 0.1f
#define CHASSIS_SPEED_COMPENSATE_PID_KI 0.1f
#define CHASSIS_SPEED_COMPENSATE_PID_KD 0.0f
#define CHASSIS_SPEED_COMPENSATE_PID_INTEGRAL_LIMIT 1.0f
#define CHASSIS_SPEED_COMPENSATE_PID_OUTPUT_LIMIT 3.0f
#define CHASSIS_FRONT_WHEEL_PID_KP 8.0f
#define CHASSIS_REAR_WHEEL_PID_KP 7.0f
#define CHASSIS_WHEEL_PID_KI 0.0f
#define CHASSIS_FRONT_WHEEL_PID_KD 2.0f
#define CHASSIS_REAR_WHEEL_PID_KD 1.5f
#define CHASSIS_WHEEL_PID_INTEGRAL_LIMIT 0.0f

#define CHASSIS_REMOTE_DEADBAND 0.1f
#define CHASSIS_REMOTE_ACCEL_STEP_MPS 0.015f
#define CHASSIS_BRAKE_STEP_MPS 0.01f
#define CHASSIS_PC_ACCEL_STEP_MPS 0.01f
#define CHASSIS_MAX_LINEAR_SPEED_MPS 3.0f
#define CHASSIS_NORM_LIMIT 1.0f
#define CHASSIS_AUTO_SLOPE_FAST_BASE_STEP_MPS 0.015f
#define CHASSIS_AUTO_SLOPE_FAST_GAIN 0.07f
#define CHASSIS_AUTO_SLOPE_FAST_START_MPS 0.25f
#define CHASSIS_AUTO_SLOPE_FAST_END_MPS 4.0f
#define CHASSIS_AUTO_SLOPE_SLOW_BASE_STEP_MPS 0.005f
#define CHASSIS_AUTO_SLOPE_SLOW_GAIN 0.005f
#define CHASSIS_AUTO_SLOPE_SLOW_START_MPS 0.65f
#define CHASSIS_AUTO_SLOPE_SLOW_END_MPS 1.5f
#define CHASSIS_AUTO_SLOPE_HIGH_GAIN 0.0085f
#define CHASSIS_AUTO_SLOPE_HIGH_START_MPS 1.35f
#define CHASSIS_SPEED_ERROR_NUMERATOR_X_MPS 0.04f
#define CHASSIS_SPEED_ERROR_NUMERATOR_Y_MPS 0.07f
#define CHASSIS_SPEED_ERROR_FEEDBACK_GAIN 0.7f
#define CHASSIS_SPEED_ERROR_DENOMINATOR_BIAS 0.5f
#define CHASSIS_SPEED_ERROR_BIAS_X_MPS 0.15f
#define CHASSIS_SPEED_ERROR_BIAS_Y_MPS 0.22f
#define CHASSIS_SLOPE_ERROR_REDUCTION_MPS 0.2f
#define CHASSIS_SLOPE_ERROR_REDUCTION_START_MPS 1.95f

#define CHASSIS_DEFAULT_REVOLVE_SPEED_RPS 0.6f
#define CHASSIS_POWER_LIMIT_50_W 50U
#define CHASSIS_POWER_LIMIT_60_W 60U
#define CHASSIS_POWER_LIMIT_70_W 70U
#define CHASSIS_POWER_LIMIT_80_W 80U
#define CHASSIS_POWER_LIMIT_100_W 100U
#define CHASSIS_POWER_LIMIT_120_W 120U
#define CHASSIS_REVOLVE_SPEED_50_W_RPS 0.65f
#define CHASSIS_REVOLVE_SPEED_60_W_RPS 0.75f
#define CHASSIS_REVOLVE_SPEED_70_W_RPS 0.85f
#define CHASSIS_REVOLVE_SPEED_80_W_RPS 0.95f
#define CHASSIS_REVOLVE_SPEED_100_W_RPS 1.10f
#define CHASSIS_REVOLVE_SPEED_120_W_RPS 1.25f
#define CHASSIS_SHIFT_REVOLVE_BOOST_RPS 0.3f
#define CHASSIS_TRANSLATION_REVOLVE_REDUCTION_RPS 0.05f
#define CHASSIS_NO_CAPACITY_REVOLVE_REDUCTION_RPS 0.05f
#define CHASSIS_MATCH_REVOLVE_HOLD_CYCLES 300U
#define CHASSIS_MATCH_REVOLVE_SPEED_RPS 0.2f
#define CHASSIS_CENTRIFUGE_COMPENSATE_MPS 0.1f
#define CHASSIS_YAW_ERROR_GAIN (-0.5f)
#define CHASSIS_HALF_TURN_D 180.0f
#define CHASSIS_FOLLOW_RETURN_TOLERANCE_D 5.0f
#define CHASSIS_RPS_TO_DPS 360.0f

#define CHASSIS_SHAKE_TOTAL_CYCLES 400U
#define CHASSIS_SHAKE_SEGMENT_CYCLES 50U
#define CHASSIS_SHAKE_ANGLE_D 45.0f
#define CHASSIS_SHAKE_POSITIVE_PHASE_1 1U
#define CHASSIS_SHAKE_POSITIVE_PHASE_3 3U
#define CHASSIS_SHAKE_POSITIVE_PHASE_5 5U
#define CHASSIS_SHAKE_POSITIVE_PHASE_7 7U
#define CHASSIS_SHAKE_POSITIVE_PHASE_8 8U
#define CHASSIS_KINEMATIC_DIVISOR 4.0f
#define CHASSIS_SQRT_TWO 1.414f

#define CHASSIS_OLD_CAPACITY_FULL_VOLTAGE_V 18.0f
#define CHASSIS_NEW_CAPACITY_FULL_VOLTAGE_V 15.0f
#define CHASSIS_CAPACITY_EMPTY_VOLTAGE_V 10.0f
#define CHASSIS_CAPACITY_ASSIST_VOLTAGE_V 10.4f
#define CHASSIS_CAPACITY_HIGH_ASSIST_VOLTAGE_V 17.4f
#define CHASSIS_OLD_MAX_COMPENSATE_POWER_W 230.0f
#define CHASSIS_NEW_MAX_COMPENSATE_POWER_W 200.0f
#define CHASSIS_CAPACITY_POWER_GAIN_W_PER_V 40.0f
#define CHASSIS_CAPACITY_CUBIC_COEFFICIENT (-0.003f)
#define CHASSIS_CAPACITY_SQUARE_COEFFICIENT 0.35f
#define CHASSIS_CAPACITY_CUBIC_EXPONENT 3.0
#define CHASSIS_CAPACITY_SQUARE_EXPONENT 2.0
#define CHASSIS_OLD_CAPACITY_POWER_BIAS_W 80.0f
#define CHASSIS_SPEED_BOOST_CYCLES 200U
#define CHASSIS_REVOLVE_EXIT_CYCLES 200U
#define CHASSIS_LOW_POWER_THRESHOLD_W 60.0f
#define CHASSIS_MID_POWER_THRESHOLD_W 70.0f
#define CHASSIS_HIGH_POWER_THRESHOLD_W 90.0f
#define CHASSIS_ACCELERATION_POWER_SCALE 1.5f
#define CHASSIS_LOW_POWER_SCALE 1.95f
#define CHASSIS_MID_POWER_SCALE 1.75f
#define CHASSIS_HIGH_POWER_SCALE 1.65f
#define CHASSIS_MAX_POWER_SCALE 1.55f
#define CHASSIS_NORMAL_POWER_SCALE 1.25f
#define CHASSIS_REVOLVE_EXIT_POWER_SCALE 0.7f
#define CHASSIS_LOW_VOLTAGE_EXIT_POWER_SCALE 0.6f
#define CHASSIS_CLIMB_POWER_SCALE 2.2f
#define CHASSIS_STAIR_BASE_POWER_W 50.0f
#define CHASSIS_STAIR_CAPACITY_SCALE 2.1f
#define CHASSIS_STAIR_NO_CAPACITY_SCALE 1.15f
#define CHASSIS_POWER_SCALE_FULL 1.0f

#define CHASSIS_WHEEL_SPEED_LPF_ALPHA 0.3f
#define CHASSIS_MOTOR_OUTPUT_LIMIT 16000.0f
#define CHASSIS_LOW_BUFFER_ENERGY_J 10.0f
#define CHASSIS_LOW_BUFFER_OUTPUT_SCALE 0.5f
#define CHASSIS_OUTPUT_NONZERO_EPSILON 1.0e-3f
#define CHASSIS_OUTCLIMB_REAR_OUTPUT_SCALE 0.8f
#define CHASSIS_CLIMB_FRONT_OUTPUT_RATIO 0.25f
#define CHASSIS_CLIMB_REAR_OUTPUT_RATIO 0.75f
#define CHASSIS_SLOPE_TRANSITION_HOLD_CYCLES 200U
#define CHASSIS_POWER_MODEL_TORQUE_COEFFICIENT 1.99688994e-6f
#define CHASSIS_POWER_MODEL_SPEED_SQUARE_COEFFICIENT 4.0e-7f
#define CHASSIS_POWER_MODEL_OUTPUT_SQUARE_COEFFICIENT 2.5e-7f
#define CHASSIS_POWER_MODEL_FIXED_LOSS_W 0.5f

typedef struct {
    ChassisControl control;
    AverageFilter power_filter;
    float wheel_speed_lpf_rpm[CHASSIS_WHEEL_COUNT];
    uint16_t match_revolve_cycles;
    uint16_t shake_cycles;
    uint16_t speed_input_cycles;
    uint16_t revolve_exit_cycles;
    uint16_t slope_transition_cycles;
    uint16_t last_chassis_mode;
    uint8_t last_slope_mode;
} chassis_runtime_t;

#define RC_CHANNEL_IDLE(channel) (fabsf(channel) < CHASSIS_REMOTE_DEADBAND)
#define RC_TRANSLATION_IDLE \
    (RC_CHANNEL_IDLE(rc_ch0) && RC_CHANNEL_IDLE(rc_ch1) && RC_CHANNEL_IDLE(rc_ch4))
#define CHASSIS_MANUAL_SOURCE_ACTIVE                         \
    (((pDecisionAO->ctrl_terminal == CONTROL_FROM_REMOTE) || \
      (pDecisionAO->ctrl_terminal == CONTROL_FROM_PC)) &&    \
     (rc_source == DT7))
#define chassis_gimbal_target_yaw_d (_gimbalControl->GimbalTargetInput.yaw_angle_d)
#define chassis_gimbal_yaw_d (_gimbalControl->GimbalEstimate.yaw_angle_d)
#define chassis_gimbal_yaw_dps (_gimbalControl->GimbalEstimate.yaw_angular_velocity_dps)

#define chassis_follow_pid (g_chassis_runtime.control.ChassisFollowControl.follow_speed_need_pid)
#define chassis_revolve_return_flag \
    (g_chassis_runtime.control.ChassisFollowControl.revolve_return_flag)
#define gimbal_speed_x_mps (g_chassis_runtime.control.GimbalCoordinateInput.speed_x_mps)
#define gimbal_speed_y_mps (g_chassis_runtime.control.GimbalCoordinateInput.speed_y_mps)
#define gimbal_max_revolve_speed_rps \
    (g_chassis_runtime.control.GimbalCoordinateInput.max_revolve_speed_rps)
#define gimbal_speed_x_compensate_pid \
    (g_chassis_runtime.control.GimbalCoordinateInput.speed_x_compensate_pid)
#define gimbal_speed_y_compensate_pid \
    (g_chassis_runtime.control.GimbalCoordinateInput.speed_y_compensate_pid)
#define chassis_target_speed_x_mps (g_chassis_runtime.control.ChassisCoordinateInput.speed_x_mps)
#define chassis_target_speed_y_mps (g_chassis_runtime.control.ChassisCoordinateInput.speed_y_mps)
#define chassis_target_speed_w_rps (g_chassis_runtime.control.ChassisCoordinateInput.speed_w_rps)
#define chassis_compensate_speed_w_dps \
    (g_chassis_runtime.control.ChassisCoordinateInput.compensate_speed_w_dps)
#define chassis_real_speed_x_mps (g_chassis_runtime.control.ChassisRealNeedInput.speed_x_mps)
#define chassis_real_speed_y_mps (g_chassis_runtime.control.ChassisRealNeedInput.speed_y_mps)
#define chassis_real_speed_w_rps (g_chassis_runtime.control.ChassisRealNeedInput.speed_w_rps)
#define chassis_power_limit_scale (g_chassis_runtime.control.ChassisRealNeedInput.power_limit_scale)
#define wheel_target_speed_mps (g_chassis_runtime.control.WheelMotorControl.target_speed_mps)
#define wheel_target_output (g_chassis_runtime.control.WheelMotorControl.target_motor_output)
#define wheel_speed_pid (g_chassis_runtime.control.WheelMotorControl.speed_control_pid)
#define chassis_gimbal_delta_angle_d \
    (g_chassis_runtime.control.ChassisEstimate.gimbal_to_chassis_delta_angle_d)
#define chassis_follow_angle_d (g_chassis_runtime.control.ChassisEstimate.chassis_follow_angle_d)
#define chassis_wheel_real_speed_mps \
    (g_chassis_runtime.control.ChassisEstimate.wheel_real_speed_mps)
#define chassis_speed_x_mps (g_chassis_runtime.control.ChassisEstimate.speed_x_mps)
#define chassis_speed_y_mps (g_chassis_runtime.control.ChassisEstimate.speed_y_mps)
#define chassis_speed_w_rps (g_chassis_runtime.control.ChassisEstimate.speed_w_rps)
#define chassis_imu_yaw_dps (g_chassis_runtime.control.ChassisEstimate.imu_yaw_dps)
#define chassis_max_compensate_power (g_chassis_runtime.control.SuperCapacity.max_compensate_power)
#define power_filter (g_chassis_runtime.power_filter)
#define wheel_speed_lpf_rpm (g_chassis_runtime.wheel_speed_lpf_rpm)
#define match_revolve_cycles (g_chassis_runtime.match_revolve_cycles)
#define shake_cycles (g_chassis_runtime.shake_cycles)
#define speed_input_cycles (g_chassis_runtime.speed_input_cycles)
#define revolve_exit_cycles (g_chassis_runtime.revolve_exit_cycles)
#define slope_transition_cycles (g_chassis_runtime.slope_transition_cycles)
#define last_chassis_mode (g_chassis_runtime.last_chassis_mode)
#define last_slope_mode (g_chassis_runtime.last_slope_mode)

#endif

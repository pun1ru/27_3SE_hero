#ifndef STIR_CONTROL_INTERNAL_H_
#define STIR_CONTROL_INTERNAL_H_

#include "stirControl.h"

#include "general_define.h"

#define SHOOT_VIRTUAL_HEAT_PER_SHOT 100U
#define STIR_REVERSE_ANGLE_D 20.0f
#define STIR_RECOVERY_TOLERANCE_D 4.0f
#define STIR_FEED_TOLERANCE_D 5.0f
#define STIR_SINGLE_STEP_ANGLE_D 60.0f
#define STIR_TWO_STEP_FIRST_ANGLE_D 20.0f
#define STIR_TWO_STEP_SECOND_ANGLE_D 40.0f
#define STIR_TWO_STEP_DELAY_CYCLES 30U
#define STIR_STALL_TORQUE_THRESHOLD_NM 8.0f
#define STIR_STALL_COUNTER_STEP 10U
#define STIR_STALL_TRIGGER_COUNT 500U
#define STIR_CALIBRATION_PERIOD_CYCLES 1000U
#define STIR_OFFLINE_CHECK_PERIOD_CYCLES 100U
#define STIR_ENCODER_WRAP_MARGIN_D 100.0f
#define STIR_CHAMBER_SPACING_D 60.0f
#define STIR_FORWARD_SELECTION_D 50.0f
#define STIR_DEGREES_PER_RADIAN 180.0f
#define STIR_LOW_SPEED_PITCH_THRESHOLD_D 0.0f
#define STIR_STOP_SPEED_RPS 0.0f
#define SHOOT_MOTOR_OUTPUT_ZERO 0
#define STIR_MOTOR_DISABLED 0U
#define STIR_MOTOR_ENABLED 1U
#define CRC16_MODBUS_INITIAL_VALUE 0xFFFFU
#define CRC16_MODBUS_POLYNOMIAL 0xA001U
#define CRC16_MODBUS_LOW_BIT_MASK 0x0001U
#define CRC16_BITS_PER_BYTE 8U
#define CRC16_SHIFT_BITS 1U

typedef enum {
    STALL_RECOVERY_IDLE = 0,
    STALL_RECOVERY_REVERSING,
    STALL_RECOVERY_RETURNING
} StirRecoveryState;

typedef enum { STIR_FEED_IDLE = 0, STIR_FEED_FIRST_STEP, STIR_FEED_SECOND_STEP } StirFeedPhase;

typedef struct {
    ShootControl control;
    float stall_preset_target_d;
    float stall_reverse_target_d;
    uint16_t virtual_heat;
    uint16_t stall_count;
    uint16_t stir_delay_cycles;
    uint16_t calibration_cycles;
    uint16_t last_frame_count;
    uint16_t previous_frame_count;
    uint16_t frame_check_cycles;
    uint8_t last_stir_block;
    uint8_t last_robot_state;
    StirRecoveryState stall_recovery_state;
    uint8_t stir_flag;
    StirFeedPhase two_step_phase;
    StirFireMode selected_fire_mode;
} shoot_runtime_t;

#define stir_motor_position_d (_stirMotorRec->pos_d)
#define stir_motor_speed_radps (_stirMotorRec->vel_radps)
#define stir_motor_torque_nm (_stirMotorRec->toq)
#define stir_motor_state (_stirMotorRec->state)
#define stir_motor_frame_count (_stirMotorRec->frame_counter)
#define shoot_heat_limit (ext_game_robot_status.shooter_barrel_heat_limit)
#define shooter_output_enabled (ext_game_robot_status.power_management_shooter_output)
#define shoot_gimbal_pitch_d (_gimbalControl->GimbalEstimate.pitch_angle_d)

#define fric_target_output (g_shoot_runtime.control.ShootMotorControl.fric_target_output)
#define stir_preset_angle_d (g_shoot_runtime.control.ShootMotorControl.stir_preset_angle)
#define stir_target_pos (g_shoot_runtime.control.ShootTargetInput.stir_target_pos)
#define stir_target_pos_rad (g_shoot_runtime.control.ShootTargetInput.stir_target_pos_rad)
#define stir_all_target_pos_d (g_shoot_runtime.control.ShootTargetInput.stir_all_target_pos_d)
#define stir_all_target_pos_rad (g_shoot_runtime.control.ShootTargetInput.stir_all_target_pos_rad)
#define stir_target_vol (g_shoot_runtime.control.ShootTargetInput.stir_target_vol)
#define shoot_flag (g_shoot_runtime.control.ShootTargetInput.shoot_flag)
#define stir_block_flag (g_shoot_runtime.control.ShootEstimate.stir_block_flag)
#define stir_reset_flag (g_shoot_runtime.control.ShootEstimate.stir_reset_flag)
#define stir_enable_desire (g_shoot_runtime.control.ShootEstimate.stir_enableflag_desire)
#define shoot_count (g_shoot_runtime.control.ShootEstimate.shoot_count)
#define stir_real_angle (g_shoot_runtime.control.ShootEstimate.stir_real_angle)
#define stir_real_angle_rad (g_shoot_runtime.control.ShootEstimate.stir_real_angle_rad)
#define stir_real_angle_d (g_shoot_runtime.control.ShootEstimate.stir_real_angle_d)
#define stir_angle_last_d (g_shoot_runtime.control.ShootEstimate.stir_angle_last)
#define stir_angle_cur_d (g_shoot_runtime.control.ShootEstimate.stir_angle_cur)
#define stir_all_angle_d (g_shoot_runtime.control.ShootEstimate.stir_all_angle_d)
#define stir_turn_count (g_shoot_runtime.control.ShootEstimate.quan_shu_r)
#define virtual_heat (g_shoot_runtime.virtual_heat)
#define stall_count (g_shoot_runtime.stall_count)
#define stir_delay_cycles (g_shoot_runtime.stir_delay_cycles)
#define stall_recovery_state (g_shoot_runtime.stall_recovery_state)
#define stir_active_flag (g_shoot_runtime.stir_flag)
#define two_step_phase (g_shoot_runtime.two_step_phase)
#define fire_mode (g_shoot_runtime.selected_fire_mode)
#define stall_preset_target_d (g_shoot_runtime.stall_preset_target_d)
#define stall_reverse_target_d (g_shoot_runtime.stall_reverse_target_d)
#define last_stir_block (g_shoot_runtime.last_stir_block)
#define last_robot_state (g_shoot_runtime.last_robot_state)
#define calibration_cycles (g_shoot_runtime.calibration_cycles)
#define last_frame_count (g_shoot_runtime.last_frame_count)
#define previous_frame_count (g_shoot_runtime.previous_frame_count)
#define frame_check_cycles (g_shoot_runtime.frame_check_cycles)

#endif

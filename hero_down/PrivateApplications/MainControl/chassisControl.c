#include "chassisControl_internal.h"

#include <math.h>
#include <stdlib.h>

#include "arm_math.h"
#include "main.h"
#include "general_config_label.h"
#include "gimbalControl.h"
#include "judge_receive.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"
#include "state_task.h"
#include "decision_ao.h"

static chassis_runtime_t g_chassis_runtime = {0};
const ChassisControl *const _chassisControl = &g_chassis_runtime.control;

static float chassis_calculate_prior_power(void);

void ChassisDecisionInitialize(void){
    PIDInitialize(&chassis_follow_pid, CHASSIS_FOLLOW_PID_KP, CHASSIS_FOLLOW_PID_KI,
                  CHASSIS_FOLLOW_PID_KD, CHASSIS_FOLLOW_PID_INTEGRAL_LIMIT,
                  CHASSIS_FOLLOW_PID_OUTPUT_LIMIT);
    PIDInitialize(&gimbal_speed_x_compensate_pid, CHASSIS_SPEED_COMPENSATE_PID_KP,
                  CHASSIS_SPEED_COMPENSATE_PID_KI, CHASSIS_SPEED_COMPENSATE_PID_KD,
                  CHASSIS_SPEED_COMPENSATE_PID_INTEGRAL_LIMIT,
                  CHASSIS_SPEED_COMPENSATE_PID_OUTPUT_LIMIT);
    PIDInitialize(&gimbal_speed_y_compensate_pid, CHASSIS_SPEED_COMPENSATE_PID_KP,
                  CHASSIS_SPEED_COMPENSATE_PID_KI, CHASSIS_SPEED_COMPENSATE_PID_KD,
                  CHASSIS_SPEED_COMPENSATE_PID_INTEGRAL_LIMIT,
                  CHASSIS_SPEED_COMPENSATE_PID_OUTPUT_LIMIT);
}

void ChassisControlInitialize(void){
    PIDInitialize(&wheel_speed_pid[LF], CHASSIS_FRONT_WHEEL_PID_KP, CHASSIS_WHEEL_PID_KI,
                  CHASSIS_FRONT_WHEEL_PID_KD, CHASSIS_WHEEL_PID_INTEGRAL_LIMIT,
                  M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&wheel_speed_pid[RF], CHASSIS_FRONT_WHEEL_PID_KP, CHASSIS_WHEEL_PID_KI,
                  CHASSIS_FRONT_WHEEL_PID_KD, CHASSIS_WHEEL_PID_INTEGRAL_LIMIT,
                  M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&wheel_speed_pid[RB], CHASSIS_REAR_WHEEL_PID_KP, CHASSIS_WHEEL_PID_KI,
                  CHASSIS_REAR_WHEEL_PID_KD, CHASSIS_WHEEL_PID_INTEGRAL_LIMIT,
                  M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&wheel_speed_pid[LB], CHASSIS_REAR_WHEEL_PID_KP, CHASSIS_WHEEL_PID_KI,
                  CHASSIS_REAR_WHEEL_PID_KD, CHASSIS_WHEEL_PID_INTEGRAL_LIMIT,
                  M3508_MAX_OUTPUT_CURRENT);
    AverageFilterInitialize(&power_filter);
}

typedef struct {
    uint8_t is_input;
    uint8_t is_x_input;
    uint8_t is_y_input;
    float max_linear_speed_x_mps;
    float max_linear_speed_y_mps;
    float max_speed_error_x_mps;
    float max_speed_error_y_mps;
    float delta_angle_sin;
    float delta_angle_cos;
} ChassisTranslationContext;

static void chassis_update_remote_translation(ChassisTranslationContext *context){
    if(RC_TRANSLATION_IDLE){
        context->is_input = CHASSIS_INPUT_INACTIVE;
    }

    if(context->is_input){
        if(fabs(gimbal_speed_x_mps - chassis_speed_x_mps) < context->max_speed_error_x_mps){
            gimbal_speed_x_mps += rc_ch0 * CHASSIS_REMOTE_ACCEL_STEP_MPS;
        }

        if(fabs(gimbal_speed_y_mps - chassis_speed_y_mps) < context->max_speed_error_y_mps){
            gimbal_speed_y_mps += rc_ch1 * CHASSIS_REMOTE_ACCEL_STEP_MPS +
                                  _robotState->auto_slope * CHASSIS_AUTO_SLOPE_FAST_BASE_STEP_MPS +
                                  _robotState->auto_slope * CHASSIS_AUTO_SLOPE_FAST_GAIN *
                                      (chassis_speed_y_mps - CHASSIS_AUTO_SLOPE_FAST_START_MPS) *
                                      (chassis_speed_y_mps > CHASSIS_AUTO_SLOPE_FAST_START_MPS &&
                                       chassis_speed_y_mps <= CHASSIS_AUTO_SLOPE_FAST_END_MPS);
        }

        if(rc_ch0 * chassis_speed_x_mps < 0.0f){
            if(gimbal_speed_x_mps * rc_ch0 < 0.0f){
                gimbal_speed_x_mps = 0.0f;
            }
            gimbal_speed_x_mps += rc_ch0 * CHASSIS_BRAKE_STEP_MPS;
        }

        if(rc_ch1 * chassis_speed_y_mps < 0.0f){
            if(gimbal_speed_y_mps * rc_ch1 < 0.0f){
                gimbal_speed_y_mps = 0.0f;
            }
            gimbal_speed_y_mps += rc_ch1 * CHASSIS_BRAKE_STEP_MPS;
        }

        gimbal_speed_x_mps =
            AbsLimiter(gimbal_speed_x_mps, context->max_linear_speed_x_mps * fabsf(rc_ch0));
        gimbal_speed_y_mps =
            AbsLimiter(gimbal_speed_y_mps,
                       context->max_linear_speed_y_mps * (fabsf(rc_ch1) + _robotState->auto_slope));
    } else {
        gimbal_speed_x_mps = chassis_speed_x_mps;
        gimbal_speed_y_mps = chassis_speed_y_mps;
    }
}

static void chassis_update_pc_translation(ChassisTranslationContext *context){
    if(key_ad_released){
        context->is_x_input = CHASSIS_INPUT_INACTIVE;
    }
    if(key_wse_released){
        context->is_y_input = CHASSIS_INPUT_INACTIVE;
    }
    if(context->is_x_input == CHASSIS_INPUT_INACTIVE &&
       context->is_y_input == CHASSIS_INPUT_INACTIVE){
        context->is_input = CHASSIS_INPUT_INACTIVE;
    }

    if(context->is_x_input){
        int8_t input_x = key_ad_direction;
        if(fabsf(gimbal_speed_x_mps - chassis_speed_x_mps) < context->max_speed_error_x_mps){
            gimbal_speed_x_mps += input_x * CHASSIS_PC_ACCEL_STEP_MPS;
        }

        if(input_x * chassis_speed_x_mps < 0.0f){
            if(gimbal_speed_x_mps * input_x < 0.0f){
                gimbal_speed_x_mps = 0.0f;
            }
            gimbal_speed_x_mps += input_x * CHASSIS_BRAKE_STEP_MPS;
        }
        gimbal_speed_x_mps = AbsLimiter(gimbal_speed_x_mps, context->max_linear_speed_x_mps);
    } else {
        gimbal_speed_x_mps = chassis_speed_x_mps;
    }

    if(context->is_y_input){
        int8_t input_y = key_ws_direction;
        if(fabsf(gimbal_speed_y_mps - chassis_speed_y_mps) < context->max_speed_error_y_mps){
            if(_robotState->auto_slope == CHASSIS_INPUT_ACTIVE){
                gimbal_speed_y_mps +=
                    _robotState->auto_slope * CHASSIS_AUTO_SLOPE_FAST_BASE_STEP_MPS +
                    _robotState->auto_slope * CHASSIS_AUTO_SLOPE_FAST_GAIN *
                        (chassis_speed_y_mps - CHASSIS_AUTO_SLOPE_FAST_START_MPS) *
                        (chassis_speed_y_mps > CHASSIS_AUTO_SLOPE_FAST_START_MPS &&
                         chassis_speed_y_mps <= CHASSIS_AUTO_SLOPE_FAST_END_MPS);
            } else {
                gimbal_speed_y_mps +=
                    input_y * CHASSIS_PC_ACCEL_STEP_MPS +
                    _robotState->auto_slope * CHASSIS_AUTO_SLOPE_SLOW_BASE_STEP_MPS +
                    _robotState->auto_slope * CHASSIS_AUTO_SLOPE_SLOW_GAIN *
                        (chassis_speed_y_mps - CHASSIS_AUTO_SLOPE_SLOW_START_MPS) *
                        (chassis_speed_y_mps > CHASSIS_AUTO_SLOPE_SLOW_START_MPS &&
                         chassis_speed_y_mps <= CHASSIS_AUTO_SLOPE_SLOW_END_MPS) +
                    _robotState->auto_slope * CHASSIS_AUTO_SLOPE_HIGH_GAIN *
                        (chassis_speed_y_mps > CHASSIS_AUTO_SLOPE_HIGH_START_MPS);
            }
        }

        if(input_y * chassis_speed_y_mps < 0.0f){
            if(gimbal_speed_y_mps * input_y < 0.0f){
                gimbal_speed_y_mps = 0.0f;
            }
            gimbal_speed_y_mps += input_y * CHASSIS_BRAKE_STEP_MPS;
        }
        gimbal_speed_y_mps = AbsLimiter(gimbal_speed_y_mps, context->max_linear_speed_y_mps);
    } else {
        gimbal_speed_y_mps = chassis_speed_y_mps;
    }
}

static void chassis_update_translation_source(ChassisTranslationContext *context){
    switch(pDecisionAO->ctrl_terminal){
        case CONTROL_STOP:
            gimbal_speed_x_mps = 0.0f;
            gimbal_speed_y_mps = 0.0f;
            chassis_revolve_return_flag = CHASSIS_INPUT_INACTIVE;
            break;

        case CONTROL_FROM_REMOTE:
            chassis_update_remote_translation(context);
            break;

        case CONTROL_FROM_PC:
            chassis_update_pc_translation(context);
            break;
    }
}

static void chassis_finalize_translation_input(const ChassisTranslationContext *context){
    float norm = context->max_linear_speed_y_mps / sqrt(gimbal_speed_x_mps * gimbal_speed_x_mps +
                                                        gimbal_speed_y_mps * gimbal_speed_y_mps);
    if(norm <= CHASSIS_NORM_LIMIT){
        gimbal_speed_x_mps *= norm;
        gimbal_speed_y_mps *= norm;
    }

    gimbal_speed_x_mps *= context->is_input * context->is_x_input;
    gimbal_speed_y_mps *= context->is_input * context->is_y_input;
    PIDRefreshBuffer(&(gimbal_speed_x_compensate_pid));
    PIDRefreshBuffer(&(gimbal_speed_y_compensate_pid));

    if(gimbal_speed_x_mps == 0.0f){
        gimbal_speed_x_mps +=
            PIDUpdate(&(gimbal_speed_x_compensate_pid), gimbal_speed_x_mps - chassis_speed_x_mps);
    } else {
        PIDRefreshBuffer(&(gimbal_speed_x_compensate_pid));
    }

#ifdef CENTRIFUGE_REVOLVE
    if(pDecisionAO->chassis_mode == CHASSIS_REVOLVE){
        gimbal_speed_y_mps += CHASSIS_CENTRIFUGE_COMPENSATE_MPS * context->delta_angle_cos;
        gimbal_speed_x_mps += CHASSIS_CENTRIFUGE_COMPENSATE_MPS * context->delta_angle_sin;
    }
#endif

    chassis_target_speed_x_mps = gimbal_speed_x_mps * context->delta_angle_cos +
                                 gimbal_speed_y_mps * context->delta_angle_sin;
    chassis_target_speed_y_mps = -gimbal_speed_x_mps * context->delta_angle_sin +
                                 gimbal_speed_y_mps * context->delta_angle_cos;
}

static uint8_t chassis_update_translation_input(void){
    ChassisTranslationContext context = {
        .is_input = CHASSIS_INPUT_ACTIVE,
        .is_x_input = CHASSIS_INPUT_ACTIVE,
        .is_y_input = CHASSIS_INPUT_ACTIVE,
        .max_linear_speed_x_mps = CHASSIS_MAX_LINEAR_SPEED_MPS,
        .max_linear_speed_y_mps = CHASSIS_MAX_LINEAR_SPEED_MPS,
        .max_speed_error_x_mps =
            CHASSIS_SPEED_ERROR_NUMERATOR_X_MPS /
                (CHASSIS_SPEED_ERROR_FEEDBACK_GAIN * fabsf(chassis_speed_x_mps) +
                 CHASSIS_SPEED_ERROR_DENOMINATOR_BIAS) +
            CHASSIS_SPEED_ERROR_BIAS_X_MPS,
        .max_speed_error_y_mps =
            CHASSIS_SPEED_ERROR_NUMERATOR_Y_MPS /
                (CHASSIS_SPEED_ERROR_FEEDBACK_GAIN * fabsf(chassis_speed_y_mps) +
                 CHASSIS_SPEED_ERROR_DENOMINATOR_BIAS) +
            CHASSIS_SPEED_ERROR_BIAS_Y_MPS -
            (_robotState->auto_slope * CHASSIS_SLOPE_ERROR_REDUCTION_MPS *
             (chassis_speed_y_mps > CHASSIS_SLOPE_ERROR_REDUCTION_START_MPS)),
    };

    arm_sin_cos_f32(-chassis_gimbal_delta_angle_d, &context.delta_angle_sin,
                    &context.delta_angle_cos);
    chassis_update_translation_source(&context);
    chassis_finalize_translation_input(&context);
    return context.is_input;
}

static void chassis_update_rotation_limit(uint8_t is_input_flag){
    gimbal_max_revolve_speed_rps = CHASSIS_DEFAULT_REVOLVE_SPEED_RPS;

    if(ext_game_robot_status.chassis_power_limit >= CHASSIS_POWER_LIMIT_120_W){
        gimbal_max_revolve_speed_rps = CHASSIS_REVOLVE_SPEED_120_W_RPS;
    } else if(ext_game_robot_status.chassis_power_limit >= CHASSIS_POWER_LIMIT_100_W){
        gimbal_max_revolve_speed_rps = CHASSIS_REVOLVE_SPEED_100_W_RPS;
    } else if(ext_game_robot_status.chassis_power_limit >= CHASSIS_POWER_LIMIT_80_W){
        gimbal_max_revolve_speed_rps = CHASSIS_REVOLVE_SPEED_80_W_RPS;
    } else if(ext_game_robot_status.chassis_power_limit >= CHASSIS_POWER_LIMIT_70_W){
        gimbal_max_revolve_speed_rps = CHASSIS_REVOLVE_SPEED_70_W_RPS;
    } else if(ext_game_robot_status.chassis_power_limit >= CHASSIS_POWER_LIMIT_60_W){
        gimbal_max_revolve_speed_rps = CHASSIS_REVOLVE_SPEED_60_W_RPS;
    } else if(ext_game_robot_status.chassis_power_limit >= CHASSIS_POWER_LIMIT_50_W){
        gimbal_max_revolve_speed_rps = CHASSIS_REVOLVE_SPEED_50_W_RPS;
    }

    if(key_shift){
        gimbal_max_revolve_speed_rps += CHASSIS_SHIFT_REVOLVE_BOOST_RPS;
    }
    if(is_input_flag != CHASSIS_INPUT_INACTIVE){
        gimbal_max_revolve_speed_rps -= CHASSIS_TRANSLATION_REVOLVE_REDUCTION_RPS;
    }
    if(_robotState->capacity_mode == NO_CAPACITY){
        gimbal_max_revolve_speed_rps -= CHASSIS_NO_CAPACITY_REVOLVE_REDUCTION_RPS;
    }

#ifdef MATCH_MODE
    if(CONTROL_FROM_REMOTE == pDecisionAO->ctrl_terminal &&
       pDecisionAO->chassis_mode == CHASSIS_REVOLVE){
        match_revolve_cycles = CHASSIS_MATCH_REVOLVE_HOLD_CYCLES;
    }
    if(match_revolve_cycles > 0U){
        match_revolve_cycles--;
        gimbal_max_revolve_speed_rps = CHASSIS_MATCH_REVOLVE_SPEED_RPS;
    }
#endif
}

static void chassis_update_rotation_target(void){
    float yaw_error_d;
    float follow_error_d;

    switch(pDecisionAO->chassis_mode){
        case CHASSIS_FOLLOW:
        case CHASSIS_FOLLOW_BACK:
            if(chassis_revolve_return_flag != CHASSIS_INPUT_INACTIVE){
                GimbalTargetAlignToEstimate();
            }

            yaw_error_d = AngleLimit(chassis_gimbal_target_yaw_d - chassis_gimbal_yaw_d,
                                     -CHASSIS_HALF_TURN_D, CHASSIS_HALF_TURN_D);
            follow_error_d = chassis_follow_angle_d + CHASSIS_YAW_ERROR_GAIN * yaw_error_d;
            chassis_target_speed_w_rps = PIDUpdate(&chassis_follow_pid, follow_error_d);

            if(chassis_revolve_return_flag != CHASSIS_INPUT_INACTIVE &&
               fabsf(AngleLimit(chassis_follow_angle_d, -CHASSIS_HALF_TURN_D,
                                CHASSIS_HALF_TURN_D)) < CHASSIS_FOLLOW_RETURN_TOLERANCE_D){
                chassis_revolve_return_flag = CHASSIS_INPUT_INACTIVE;
            }
            break;

        case CHASSIS_REVOLVE:
            if(CHASSIS_MANUAL_SOURCE_ACTIVE){
                chassis_target_speed_w_rps = -gimbal_max_revolve_speed_rps;
                chassis_revolve_return_flag = Sign(chassis_speed_w_rps);
            }
            break;

        case CHASSIS_SEPARATE:
            chassis_target_speed_w_rps = 0.0f;
            break;
    }
}

static void chassis_update_rotation_observer(void){
    chassis_imu_yaw_dps = chassis_gimbal_yaw_dps;
    chassis_compensate_speed_w_dps =
        CHASSIS_RPS_TO_DPS * chassis_target_speed_w_rps - chassis_imu_yaw_dps;
}

static void chassis_update_rotation_input(uint8_t is_input_flag){
    chassis_update_rotation_limit(is_input_flag);
    chassis_update_rotation_target();
    chassis_update_rotation_observer();

    chassis_target_speed_w_rps = limiter(chassis_target_speed_w_rps, gimbal_max_revolve_speed_rps);
}

static void chassis_apply_input_guards(void){
    if(CONTROL_STOP == pDecisionAO->ctrl_terminal){
        chassis_target_speed_w_rps = 0.0f;
    }

    if(pDecisionAO->sniper == SNIPER_ON){
        chassis_target_speed_x_mps = 0.0f;
        chassis_target_speed_y_mps = 0.0f;
        chassis_target_speed_w_rps = 0.0f;
        PIDRefreshBuffer(&gimbal_speed_x_compensate_pid);
        PIDRefreshBuffer(&gimbal_speed_y_compensate_pid);
    }
}

void ChassisInputUpdate(void){
    uint8_t is_input_flag = chassis_update_translation_input();

    chassis_update_rotation_input(is_input_flag);
    chassis_apply_input_guards();
}

/**
 * @brief 底盘相关观测数据更新
 */
static void chassis_update_gimbal_relation(void){
    uint16_t shake_phase;

    chassis_gimbal_delta_angle_d =
        AngleLimit(-gimbal_yaw_rx_d, -CHASSIS_HALF_TURN_D, CHASSIS_HALF_TURN_D);
    chassis_follow_angle_d = chassis_gimbal_delta_angle_d;
    if(pDecisionAO->chassis_mode == CHASSIS_FOLLOW_BACK){
        chassis_follow_angle_d += CHASSIS_HALF_TURN_D;
    }

    if(pDecisionAO->sniper == SNIPER_OFF && key_ctrl){
        shake_cycles = CHASSIS_SHAKE_TOTAL_CYCLES;
    }
    if(shake_cycles > 0U){
        shake_cycles--;
        shake_phase = shake_cycles / CHASSIS_SHAKE_SEGMENT_CYCLES;
        switch(shake_phase){
            case CHASSIS_SHAKE_POSITIVE_PHASE_1:
            case CHASSIS_SHAKE_POSITIVE_PHASE_3:
            case CHASSIS_SHAKE_POSITIVE_PHASE_5:
            case CHASSIS_SHAKE_POSITIVE_PHASE_7:
            case CHASSIS_SHAKE_POSITIVE_PHASE_8:
                chassis_follow_angle_d += CHASSIS_SHAKE_ANGLE_D;
                break;

            default:
                chassis_follow_angle_d -= CHASSIS_SHAKE_ANGLE_D;
                break;
        }
    }

    chassis_follow_angle_d =
        AngleLimit(chassis_follow_angle_d, -CHASSIS_HALF_TURN_D, CHASSIS_HALF_TURN_D);
}

static void chassis_update_velocity_estimate(void){
    float chassis_coordinate_speed_x_mps;
    float chassis_coordinate_speed_y_mps;
    float delta_angle_cos;
    float delta_angle_sin;

    for(uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++){
        chassis_wheel_real_speed_mps[i] =
            -_chassisMotorRec[i].mechanical_speed_rpm * WHEEL_RPM_TO_WHEEL_MPS;
    }

    chassis_speed_w_rps = (chassis_wheel_real_speed_mps[LF] + chassis_wheel_real_speed_mps[RF] +
                           chassis_wheel_real_speed_mps[LB] + chassis_wheel_real_speed_mps[RB]) /
                          CHASSIS_KINEMATIC_DIVISOR * WHEEL_MPS_TO_ROBOT_RPS;
    chassis_coordinate_speed_x_mps =
        (chassis_wheel_real_speed_mps[LF] + chassis_wheel_real_speed_mps[RF] -
         chassis_wheel_real_speed_mps[LB] - chassis_wheel_real_speed_mps[RB]) /
        CHASSIS_KINEMATIC_DIVISOR / CHASSIS_SQRT_TWO;
    chassis_coordinate_speed_y_mps =
        (chassis_wheel_real_speed_mps[LF] - chassis_wheel_real_speed_mps[RF] +
         chassis_wheel_real_speed_mps[LB] - chassis_wheel_real_speed_mps[RB]) /
        CHASSIS_KINEMATIC_DIVISOR / CHASSIS_SQRT_TWO;

    arm_sin_cos_f32(chassis_gimbal_delta_angle_d, &delta_angle_sin, &delta_angle_cos);
    chassis_speed_x_mps = chassis_coordinate_speed_x_mps * delta_angle_cos -
                          chassis_coordinate_speed_y_mps * delta_angle_sin;
    chassis_speed_y_mps = chassis_coordinate_speed_x_mps * delta_angle_sin +
                          chassis_coordinate_speed_y_mps * delta_angle_cos;
}

void ChassisEstimateUpdate(void){
    chassis_update_gimbal_relation();
    chassis_update_velocity_estimate();
}

/**
 * @brief 底盘闭环控制
 */

static void chassis_prepare_control_target(void){
    chassis_real_speed_x_mps = chassis_target_speed_x_mps;
    chassis_real_speed_y_mps = chassis_target_speed_y_mps;
    chassis_real_speed_w_rps = chassis_target_speed_w_rps;

    if(pDecisionAO->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB){
        if(RC_CHANNEL_IDLE(rc_ch1)){
            chassis_real_speed_y_mps = 0.0f;
        }
        chassis_real_speed_x_mps = 0.0f;
        chassis_real_speed_w_rps = 0.0f;
    }

    wheel_target_speed_mps[LF] = chassis_real_speed_y_mps * CHASSIS_SQRT_TWO +
                                 chassis_real_speed_x_mps * CHASSIS_SQRT_TWO +
                                 chassis_real_speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;
    wheel_target_speed_mps[RF] = -chassis_real_speed_y_mps * CHASSIS_SQRT_TWO +
                                 chassis_real_speed_x_mps * CHASSIS_SQRT_TWO +
                                 chassis_real_speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;
    wheel_target_speed_mps[LB] = chassis_real_speed_y_mps * CHASSIS_SQRT_TWO -
                                 chassis_real_speed_x_mps * CHASSIS_SQRT_TWO +
                                 chassis_real_speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;
    wheel_target_speed_mps[RB] = -chassis_real_speed_y_mps * CHASSIS_SQRT_TWO -
                                 chassis_real_speed_x_mps * CHASSIS_SQRT_TWO +
                                 chassis_real_speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;

    for(uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++){
        wheel_target_speed_mps[i] = -wheel_target_speed_mps[i];
    }
}

static void chassis_update_max_compensate_power(void){
#ifdef OLD_CAPACITY
    if(_superCapacity->cap_volt >= CHASSIS_OLD_CAPACITY_FULL_VOLTAGE_V){
        chassis_max_compensate_power = CHASSIS_OLD_MAX_COMPENSATE_POWER_W;
    } else {
        chassis_max_compensate_power =
            CHASSIS_CAPACITY_CUBIC_COEFFICIENT *
                pow(_superCapacity->cap_volt, CHASSIS_CAPACITY_CUBIC_EXPONENT) +
            CHASSIS_CAPACITY_SQUARE_COEFFICIENT *
                pow(_superCapacity->cap_volt, CHASSIS_CAPACITY_SQUARE_EXPONENT) +
            CHASSIS_OLD_CAPACITY_POWER_BIAS_W;
    }
#else
    if(_superCapacity->cap_volt >= CHASSIS_NEW_CAPACITY_FULL_VOLTAGE_V){
        chassis_max_compensate_power = CHASSIS_NEW_MAX_COMPENSATE_POWER_W;
    } else if(_superCapacity->cap_volt > CHASSIS_CAPACITY_EMPTY_VOLTAGE_V &&
              _superCapacity->cap_volt < CHASSIS_NEW_CAPACITY_FULL_VOLTAGE_V){
        chassis_max_compensate_power =
            CHASSIS_CAPACITY_POWER_GAIN_W_PER_V *
            (_superCapacity->cap_volt - CHASSIS_CAPACITY_EMPTY_VOLTAGE_V);
    } else {
        chassis_max_compensate_power = 0.0f;
    }
#endif
}

static void chassis_update_power_counters(uint16_t *speed_count, uint16_t *revolve_exit_count){
    speed_input_cycles++;
    if(key_wasd_released && pDecisionAO->chassis_mode != CHASSIS_REVOLVE){
        speed_input_cycles = 0U;
    }

    if(pDecisionAO->chassis_mode != CHASSIS_REVOLVE && last_chassis_mode == CHASSIS_REVOLVE){
        revolve_exit_cycles = CHASSIS_REVOLVE_EXIT_CYCLES;
    }
    if(revolve_exit_cycles > 0U){
        revolve_exit_cycles--;
    }
    last_chassis_mode = pDecisionAO->chassis_mode;

    *speed_count = speed_input_cycles;
    *revolve_exit_count = revolve_exit_cycles;
}

static float chassis_calculate_permitted_power(uint16_t speed_count, uint16_t revolve_exit_count){
    float permitted_power = ext_game_robot_status.chassis_power_limit;

    if(_superCapacity->cap_volt >= CHASSIS_CAPACITY_ASSIST_VOLTAGE_V){
        if(key_shift){
            permitted_power += CHASSIS_CAPACITY_CUBIC_COEFFICIENT *
                                   pow(_superCapacity->cap_volt, CHASSIS_CAPACITY_CUBIC_EXPONENT) +
                               CHASSIS_CAPACITY_SQUARE_COEFFICIENT *
                                   pow(_superCapacity->cap_volt, CHASSIS_CAPACITY_SQUARE_EXPONENT);
        } else if(speed_count < CHASSIS_SPEED_BOOST_CYCLES){
            permitted_power *= CHASSIS_ACCELERATION_POWER_SCALE;
        } else if(_superCapacity->cap_volt >= CHASSIS_CAPACITY_HIGH_ASSIST_VOLTAGE_V){
            if(permitted_power <= CHASSIS_LOW_POWER_THRESHOLD_W){
                permitted_power *= CHASSIS_LOW_POWER_SCALE;
            } else if(permitted_power == CHASSIS_MID_POWER_THRESHOLD_W){
                permitted_power *= CHASSIS_MID_POWER_SCALE;
            } else if(permitted_power <= CHASSIS_HIGH_POWER_THRESHOLD_W){
                permitted_power *= CHASSIS_HIGH_POWER_SCALE;
            } else {
                permitted_power *= CHASSIS_MAX_POWER_SCALE;
            }
        } else {
            permitted_power *= CHASSIS_NORMAL_POWER_SCALE;
        }

        if(revolve_exit_count > 0U){
            permitted_power *= CHASSIS_REVOLVE_EXIT_POWER_SCALE;
        }
    } else if(revolve_exit_count > 0U){
        permitted_power *= CHASSIS_LOW_VOLTAGE_EXIT_POWER_SCALE;
    }

    if(pDecisionAO->joint_mode == ROBOT_JOINT_MODE_CLIMB){
        permitted_power *= CHASSIS_CLIMB_POWER_SCALE;
    } else if(pDecisionAO->stand_mode == ROBOT_STAND_MODE_PRE_STAIR ||
              pDecisionAO->stand_mode == ROBOT_STAND_MODE_STAIR_UP){
        if(_robotState->capacity_mode != NO_CAPACITY &&
           _superCapacity->cap_volt >= CHASSIS_CAPACITY_ASSIST_VOLTAGE_V){
            permitted_power = CHASSIS_STAIR_BASE_POWER_W * CHASSIS_STAIR_CAPACITY_SCALE;
        } else {
            permitted_power = CHASSIS_STAIR_BASE_POWER_W * CHASSIS_STAIR_NO_CAPACITY_SCALE;
        }
    } else if(_robotState->capacity_mode == NO_CAPACITY){
        permitted_power = ext_game_robot_status.chassis_power_limit;
    }

    return DoubleEdgeLimiter(permitted_power, 0.0f,
                             ext_game_robot_status.chassis_power_limit +
                                 chassis_max_compensate_power);
}

static void chassis_update_power_limit(void){
    uint16_t speed_count;
    uint16_t revolve_exit_count;
    float prior_power = chassis_calculate_prior_power();

    chassis_power_limit_scale = CHASSIS_POWER_SCALE_FULL;
    chassis_update_max_compensate_power();
    chassis_update_power_counters(&speed_count, &revolve_exit_count);

    float permitted_power = chassis_calculate_permitted_power(speed_count, revolve_exit_count);
    if(prior_power > permitted_power){
        chassis_power_limit_scale = permitted_power / prior_power;
    }
}

static void chassis_update_wheel_outputs(void){
    for(uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++){
        wheel_speed_lpf_rpm[i] =
            CHASSIS_WHEEL_SPEED_LPF_ALPHA * _chassisMotorRec[i].mechanical_speed_rpm +
            (CHASSIS_POWER_SCALE_FULL - CHASSIS_WHEEL_SPEED_LPF_ALPHA) * wheel_speed_lpf_rpm[i];

        wheel_target_output[i] =
            PIDUpdate(&wheel_speed_pid[i],
                      wheel_target_speed_mps[i] * WHEEL_MPS_TO_WHEEL_RPM - wheel_speed_lpf_rpm[i]);
        wheel_target_output[i] =
            AbsLimiter(wheel_target_output[i] +
                           wheel_target_speed_mps[i] * CHASSIS_MOTOR_FEEDFORWARD_OUTPUT_PER_MPS,
                       CHASSIS_MOTOR_OUTPUT_LIMIT) *
            chassis_power_limit_scale;
        if(ext_power_heat_data.buffer_energy < CHASSIS_LOW_BUFFER_ENERGY_J){
            wheel_target_output[i] *= CHASSIS_LOW_BUFFER_OUTPUT_SCALE;
        }
    }
}

static void chassis_apply_special_output_allocation(void){
    if(pDecisionAO->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB){
        float total_output = abs(wheel_target_output[LF]) + abs(wheel_target_output[RF]) +
                             abs(wheel_target_output[LB]) + abs(wheel_target_output[RB]);
        float rear_output = abs(wheel_target_output[LB]) + abs(wheel_target_output[RB]);
        float scale =
            (rear_output > CHASSIS_OUTPUT_NONZERO_EPSILON) ? (total_output / rear_output) : 0.0f;

        wheel_target_output[LF] = CHASSIS_ZERO_OUTPUT;
        wheel_target_output[RF] = CHASSIS_ZERO_OUTPUT;
        wheel_target_output[LB] =
            AbsLimiter(wheel_target_output[LB] * scale * CHASSIS_OUTCLIMB_REAR_OUTPUT_SCALE,
                       CHASSIS_MOTOR_OUTPUT_LIMIT);
        wheel_target_output[RB] =
            AbsLimiter(wheel_target_output[RB] * scale * CHASSIS_OUTCLIMB_REAR_OUTPUT_SCALE,
                       CHASSIS_MOTOR_OUTPUT_LIMIT);
    }
    /* 上坡模式(CLIMB)：前腿功率1:3分配到后腿 */
    if(pDecisionAO->joint_mode == ROBOT_JOINT_MODE_CLIMB){
        float front_sum = (float)abs(wheel_target_output[LF]) + (float)abs(wheel_target_output[RF]);
        float rear_sum = (float)abs(wheel_target_output[LB]) + (float)abs(wheel_target_output[RB]);
        float total = front_sum + rear_sum;

        if(total > CHASSIS_OUTPUT_NONZERO_EPSILON){
            float front_scale = (front_sum > CHASSIS_OUTPUT_NONZERO_EPSILON)
                                    ? (total * CHASSIS_CLIMB_FRONT_OUTPUT_RATIO / front_sum)
                                    : 0.0f;
            float rear_scale = (rear_sum > CHASSIS_OUTPUT_NONZERO_EPSILON)
                                   ? (total * CHASSIS_CLIMB_REAR_OUTPUT_RATIO / rear_sum)
                                   : 0.0f;

            wheel_target_output[LF] = (int16_t)AbsLimiter(
                (float)wheel_target_output[LF] * front_scale, CHASSIS_MOTOR_OUTPUT_LIMIT);
            wheel_target_output[RF] = (int16_t)AbsLimiter(
                (float)wheel_target_output[RF] * front_scale, CHASSIS_MOTOR_OUTPUT_LIMIT);
            wheel_target_output[LB] = (int16_t)AbsLimiter(
                (float)wheel_target_output[LB] * rear_scale, CHASSIS_MOTOR_OUTPUT_LIMIT);
            wheel_target_output[RB] = (int16_t)AbsLimiter(
                (float)wheel_target_output[RB] * rear_scale, CHASSIS_MOTOR_OUTPUT_LIMIT);
        }
    }
}

static void chassis_apply_slope_transition(void){
    uint8_t current_slope = _robotState->auto_slope;

    if(slope_transition_cycles > 0U){
        slope_transition_cycles--;
        wheel_target_output[LF] = wheel_target_output[RF] = CHASSIS_ZERO_OUTPUT;
    }
    if(CONTROL_FROM_REMOTE == pDecisionAO->ctrl_terminal && current_slope == 0U &&
       current_slope != last_slope_mode){
        slope_transition_cycles = CHASSIS_SLOPE_TRANSITION_HOLD_CYCLES;
    }
    last_slope_mode = current_slope;
}

static void chassis_apply_output_guards(void){
#if defined CHASSIS_OFF
    for(uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++){
        wheel_target_output[i] = CHASSIS_ZERO_OUTPUT;
    }
#endif

    if(CONTROL_STOP == pDecisionAO->ctrl_terminal){
        for(uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++){
            wheel_target_output[i] = CHASSIS_ZERO_OUTPUT;
        }
    }
}

void ChassisControlUpdate(void){
    chassis_prepare_control_target();
    chassis_update_power_limit();
    chassis_update_wheel_outputs();
    chassis_apply_special_output_allocation();
    chassis_apply_slope_transition();
    chassis_apply_output_guards();
}

static float chassis_calculate_prior_power(void){
    float prior_chassis_power = 0.0f;
    int16_t target_motor_output = CHASSIS_ZERO_OUTPUT;
    float prior_motor_power = 0.0f;

    for(uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++){
        target_motor_output =
            PIDUpdatePrior(wheel_speed_pid[i], (wheel_target_speed_mps[i] * WHEEL_MPS_TO_WHEEL_RPM -
                                                _chassisMotorRec[i].mechanical_speed_rpm)) +
            wheel_target_speed_mps[i] * CHASSIS_MOTOR_FEEDFORWARD_OUTPUT_PER_MPS;

        target_motor_output = AbsLimiter(target_motor_output, CHASSIS_MOTOR_OUTPUT_LIMIT);

        prior_motor_power =
            target_motor_output * _chassisMotorRec[i].mechanical_speed_rpm *
                CHASSIS_POWER_MODEL_TORQUE_COEFFICIENT +
            CHASSIS_POWER_MODEL_SPEED_SQUARE_COEFFICIENT *
                square(_chassisMotorRec[i].mechanical_speed_rpm) +
            CHASSIS_POWER_MODEL_OUTPUT_SQUARE_COEFFICIENT * square(target_motor_output) +
            CHASSIS_POWER_MODEL_FIXED_LOSS_W;
        prior_chassis_power += prior_motor_power;
    }
    prior_chassis_power = AverageFilterUpdate(&power_filter, prior_chassis_power);
    return prior_chassis_power;
}

/**
 * @file gimbalControl.c
 * @brief 云台输入、姿态估计和闭环控制
 */

#include "gimbalControl_internal.h"

#include <math.h>
#include <string.h>

#include "Board2Board.h"
#include "decision_ao.h"
#include "DMJ4310.h"
#include "task_receive.h"
#include "worldGimbal.h"

/* 注释此宏使用 LADRC，启用后使用 LTD + 双环 PID。 */
/* #define YAW_DUAL_PID */

static gimbal_runtime_t g_gimbal_runtime = {0};
const GimbalControl *const _gimbalControl = &g_gimbal_runtime.control;
const float yaw_dm_forward_offset_rad = GIMBAL_YAW_DM_FORWARD_OFFSET_RAD;

extern int64_t circle_angle;
extern volatile float g_b2b_yaw_cmd_d;
extern volatile float g_b2b_pitch_cmd_d;
extern volatile uint8_t g_b2b_yaw_cmd_valid;

static void reset_pitch_controller(void);
static void reset_yaw_controller(void);
static void update_pitch_position_command(void);
static void sync_world_gimbal_mode(void);
static void initialize_pitch_controller(void);
static void initialize_yaw_controller(void);

static void reset_pitch_controller(void){
    pitch_ltd.error_sum = 0.0f;
    pitch_controller.eso.z1 = 0.0f;
    pitch_controller.eso.z2 = 0.0f;
    pitch_controller.eso.z3 = 0.0f;
    pitch_controller.esf.output = 0.0f;
    pitch_controller.u_0 = 0.0f;
    pitch_controller.u = 0.0f;
    pitch_target_output = 0;
}

static void reset_yaw_controller(void){
#ifdef YAW_DUAL_PID
    LTD_Reset(&yaw_ltd, gimbal_yaw_d);
    PIDReset(&yaw_speed_pid);
    yaw_ltd.error_sum = 0.0f;
#else
    float yaw_reset_rad = gimbal_yaw_d * GIMBAL_DEG_TO_RAD;
    TD_Reset(&yaw_controller.td, yaw_reset_rad);
    ESO_Reset(&yaw_controller.eso, yaw_reset_rad);
#endif
    yaw_target_output = 0.0f;
}

static void update_pitch_position_command(void){
    float encoder_target = gimbal_target_pitch_d * GIMBAL_PITCH_ENCODER_SCALE;
    int32_t encoder_position;

    encoder_target = DoubleEdgeLimiter(encoder_target,
                                       GIMBAL_PITCH_ENCODER_MIN,
                                       GIMBAL_PITCH_ENCODER_MAX);
    encoder_position = (int32_t)GIMBAL_PITCH_ENCODER_ZERO + (int32_t)encoder_target;
    sniper_position = (uint32_t)encoder_position;
    sniper_spin_direction = (sniper_position > circle_angle) ? GIMBAL_SPIN_CLOCKWISE : GIMBAL_SPIN_COUNTERCLOCKWISE;
    sniper_max_speed_dps = GIMBAL_SNIPER_MAX_SPEED_DPS;
}

static void sync_world_gimbal_mode(void){
    uint8_t desired_enable =
        (pDecisionAO->world_enable == WORLD_ENABLE_ON) ? WORLD_GIMBAL_ENABLED : WORLD_GIMBAL_DISABLED;

    if(desired_enable == _worldGimbal->enable){
        return;
    }
    if(desired_enable == WORLD_GIMBAL_ENABLED){
        WorldGimbalAlignToCurrent();
    }
    WorldGimbalSetEnabled(desired_enable);
}

static void initialize_pitch_controller(void){
    float td_init[3] = {0.0f, GIMBAL_PITCH_CONTROL_PERIOD_S, 0.0f};
    float lesf_init[5] = {
        0.0f,
        GIMBAL_PITCH_ESF_KP,
        GIMBAL_PITCH_ESF_KD,
        GIMBAL_PITCH_ESF_ERROR_LIMIT,
        GIMBAL_PITCH_ESF_OUTPUT_LIMIT};
    float eso_init[6] = {
        GIMBAL_PITCH_CONTROL_PERIOD_S,
        GIMBAL_PITCH_ESO_INPUT_GAIN,
        GIMBAL_ESO_SECOND_ORDER_COEFFICIENT * GIMBAL_PITCH_ESO_WO,
        GIMBAL_ESO_SECOND_ORDER_COEFFICIENT * GIMBAL_PITCH_ESO_WO *
            GIMBAL_PITCH_ESO_WO,
        GIMBAL_PITCH_ESO_Z3_GAIN * GIMBAL_PITCH_ESO_WO *
            GIMBAL_PITCH_ESO_WO * GIMBAL_PITCH_ESO_WO,
        GIMBAL_PITCH_ESO_Z3_LIMIT};

    LTDInitialize(&pitch_ltd,
                  GIMBAL_PITCH_LTD_R,
                  GIMBAL_PITCH_LTD_H,
                  GIMBAL_PITCH_LTD_MIN_D,
                  GIMBAL_PITCH_LTD_MAX_D);
    pitch_ltd.ki1 = GIMBAL_PITCH_LTD_KI;
    pitch_ltd.error_sum = 0.0f;
    pitch_ltd.error_sum_max = GIMBAL_PITCH_LTD_ERROR_SUM_MAX;
    pitch_ltd.lv_bo = GIMBAL_PITCH_LTD_FILTER;

    LADRCInitialize(&pitch_controller,
                    td_init,
                    lesf_init,
                    eso_init,
                    0.0f,
                    0.0f);
    ESOSetStateLimit(&pitch_controller.eso,
                     PITCH_ESO_Z1_MIN_D,
                     PITCH_ESO_Z1_MAX_D,
                     PITCH_ESO_Z2_MIN_DPS,
                     PITCH_ESO_Z2_MAX_DPS);
    ADRCBindTrackDiffLTD(&pitch_controller, &pitch_ltd);
    ADRCBindVelocityRef(&pitch_controller, ADRCVelocityFromPitchEstimate);
}

static void initialize_yaw_controller(void){
    float td_init[3] = {
        GIMBAL_YAW_TD_R,
        GIMBAL_YAW_CONTROL_PERIOD_S,
        GIMBAL_YAW_TD_GAIN};
    float lesf_init[5] = {
        0.0f,
        GIMBAL_YAW_ESF_WC * GIMBAL_YAW_ESF_WC,
        GIMBAL_ESF_DAMPING_COEFFICIENT * GIMBAL_YAW_ESF_WC,
        0.0f,
        GIMBAL_YAW_ESF_OUTPUT_LIMIT};
    float eso_init[6] = {
        GIMBAL_YAW_CONTROL_PERIOD_S,
        GIMBAL_YAW_ESO_INPUT_GAIN,
        GIMBAL_ESO_SECOND_ORDER_COEFFICIENT * GIMBAL_YAW_ESO_WO,
        GIMBAL_ESO_SECOND_ORDER_COEFFICIENT * GIMBAL_YAW_ESO_WO *
            GIMBAL_YAW_ESO_WO,
        GIMBAL_YAW_ESO_WO * GIMBAL_YAW_ESO_WO * GIMBAL_YAW_ESO_WO,
        GIMBAL_YAW_ESO_Z3_LIMIT};

    LADRCInitialize(&yaw_controller,
                    td_init,
                    lesf_init,
                    eso_init,
                    GIMBAL_YAW_MIN_RAD,
                    GIMBAL_YAW_MAX_RAD);
    ESOSetFalTuning(&yaw_controller.eso,
                    YAW_ESO_FAL_DELTA_GAIN_Z2,
                    ESO_FAL_DELTA_GAIN_UNIT);
    ScalarKalmanFilterInit(&yaw_encoder_kalman_filter,
                           GIMBAL_YAW_KF_PROCESS_NOISE,
                           GIMBAL_YAW_KF_MEASUREMENT_NOISE,
                           GIMBAL_YAW_CONTROL_PERIOD_S);

#ifdef YAW_DUAL_PID
    LTDInitialize(&yaw_ltd,
                  GIMBAL_YAW_LTD_R,
                  GIMBAL_YAW_CONTROL_PERIOD_S,
                  -GIMBAL_HALF_TURN_D,
                  GIMBAL_HALF_TURN_D);
    PIDInitialize(&yaw_position_pid,
                  GIMBAL_YAW_POSITION_KP,
                  GIMBAL_YAW_POSITION_KI,
                  0.0f,
                  GIMBAL_YAW_POSITION_OUTPUT_LIMIT,
                  GIMBAL_YAW_POSITION_INTEGRAL_LIMIT);
    PIDInitialize(&yaw_speed_pid,
                  GIMBAL_YAW_SPEED_KP,
                  0.0f,
                  GIMBAL_YAW_SPEED_KD,
                  0.0f,
                  GIMBAL_YAW_SPEED_OUTPUT_LIMIT);
#endif
}

void GimbalControlInitialize(void){
    memset(&g_gimbal_runtime, 0, sizeof(g_gimbal_runtime));
    gimbal_target_pitch_d = GIMBAL_PITCH_INITIAL_TARGET_D;
    yaw_kf_realign = GIMBAL_YAW_KF_REALIGN_REQUIRED;
    previous_sniper_mode = SNIPER_OFF;

    initialize_pitch_controller();
    initialize_yaw_controller();
    WorldGimbalInitialize();
}

void GimbalInputUpdate(void){
    sync_world_gimbal_mode();

    switch(pDecisionAO->ctrl_terminal){
        case CONTROL_STOP:
            gimbal_target_yaw_d = gimbal_yaw_d;
            gimbal_target_pitch_d = gimbal_pitch_d;
            gimbal_target_yaw_dps = 0.0f;
            break;

        case CONTROL_FROM_REMOTE:
        case CONTROL_FROM_PC:
            if(g_b2b_yaw_cmd_valid && g_b2b_down_valid){
                gimbal_target_yaw_d = g_b2b_yaw_cmd_d;
                gimbal_target_pitch_d = g_b2b_pitch_cmd_d;
            } else {
                gimbal_target_yaw_d = gimbal_yaw_d;
                gimbal_target_pitch_d = gimbal_pitch_d;
            }
            break;

        default:
            break;
    }

    gimbal_target_yaw_d = AngleLimit(gimbal_target_yaw_d,
                                     -GIMBAL_HALF_TURN_D,
                                     GIMBAL_HALF_TURN_D);
    gimbal_target_pitch_d = DoubleEdgeLimiter(gimbal_target_pitch_d,
                                              GIMBAL_PITCH_LOWER_LIMIT_D,
                                              GIMBAL_PITCH_UPPER_LIMIT_D);
}

void GimbalPoseUpdate(float pitch_angle_d, float pitch_angular_velocity_radps,
                      float yaw_angle_d, float yaw_angular_velocity_radps,
                      float roll_angle_d, float roll_angular_velocity_dps){
    gimbal_roll_d = roll_angle_d;
    gimbal_roll_dps = roll_angular_velocity_dps;
    pitch_angle_from_encoder_d =
        ((float)_pitchMotorRec->mechanical_angle - PITCH_OFFSET_MACHENICAL_ANGLE) *
        GIMBAL_FULL_ROTATION_D / LK_FULL_CIRCLE_MECHENICAL_ANGLE;
    pitch_angle_from_encoder_d = AngleLimit(pitch_angle_from_encoder_d,
                                            -GIMBAL_HALF_TURN_D,
                                            GIMBAL_HALF_TURN_D);

    if(pDecisionAO->sniper == SNIPER_ON){
        float yaw_encoder_rad = _DMyawMotorRec->pos_d - yaw_dm_forward_offset_rad;
        float yaw_encoder_dps = -_DMyawMotorRec->vel_radps * GIMBAL_RAD_TO_DEG;

        gimbal_yaw_dps = yaw_encoder_dps;
        if(isfinite(yaw_encoder_rad)){
            float measurement_d = AngleLimit(yaw_encoder_rad * GIMBAL_RAD_TO_DEG,
                                             -GIMBAL_HALF_TURN_D,
                                             GIMBAL_HALF_TURN_D);
            float velocity_dps = isfinite(yaw_encoder_dps) ? yaw_encoder_dps : 0.0f;

            if(yaw_kf_realign == GIMBAL_YAW_KF_REALIGN_REQUIRED){
                ScalarKalmanFilterReset(&yaw_encoder_kalman_filter, measurement_d);
                yaw_kf_realign = GIMBAL_YAW_KF_ALIGNED;
            }
            gimbal_yaw_d = ScalarKalmanFilterUpdate(&yaw_encoder_kalman_filter,
                                                    measurement_d,
                                                    velocity_dps);
        }
    } else {
        yaw_kf_realign = GIMBAL_YAW_KF_REALIGN_REQUIRED;
        gimbal_yaw_d = yaw_angle_d;
        gimbal_yaw_dps = yaw_angular_velocity_radps * GIMBAL_RAD_TO_DEG;
    }

    if(previous_sniper_mode != pDecisionAO->sniper){
        pose_warmup_cycles = 0U;
        gimbal_target_yaw_d = gimbal_yaw_d;
        reset_yaw_controller();
        previous_sniper_mode = pDecisionAO->sniper;
    }

    if(pose_warmup_cycles < GIMBAL_POSE_WARMUP_CYCLES){
        pose_warmup_cycles++;
    }

    gimbal_pitch_dps = pitch_angular_velocity_radps * GIMBAL_RAD_TO_DEG;
    if(pDecisionAO->sniper == SNIPER_OFF){
        gimbal_pitch_d = pitch_angle_d;
    } else if(pDecisionAO->sniper == SNIPER_ON){
        gimbal_pitch_d = pitch_angle_from_encoder_d;
    }
}

void GimbalControlUpdate(void){
    ADRCUpdate(&pitch_controller, gimbal_target_pitch_d, gimbal_pitch_d);
    if(pDecisionAO->sniper == SNIPER_ON){
        update_pitch_position_command();
    } else {
        int pitch_output = (int)pitch_controller.u;
        pitch_target_output = (int16_t)AbsLimiter(pitch_output,
                                                  GIMBAL_PITCH_OUTPUT_LIMIT);
    }

#ifdef YAW_DUAL_PID
    {
        float yaw_position_error_d = AngleLimit(yaw_ltd.x1 - gimbal_yaw_d,
                                                -GIMBAL_HALF_TURN_D,
                                                GIMBAL_HALF_TURN_D);
        float yaw_torque_feedforward = 0.0f;

        LTDUpdate(&yaw_ltd, gimbal_target_yaw_d);
        PIDUpdate(&yaw_position_pid, yaw_position_error_d);
        PIDUpdate(&yaw_speed_pid, yaw_position_pid.output - gimbal_yaw_dps);
        yaw_target_output = -(yaw_speed_pid.output + yaw_torque_feedforward);
        g_gimbal_runtime.control.GimbalMotorControl.w_d = yaw_position_pid.output;
        gimbal_target_yaw_dps = yaw_position_pid.output;
    }
#else
    ADRCUpdate(&yaw_controller,
               gimbal_target_yaw_d * GIMBAL_DEG_TO_RAD,
               gimbal_yaw_d * GIMBAL_DEG_TO_RAD);
    yaw_target_output = -yaw_controller.u;
#endif

    if((pDecisionAO->ctrl_terminal == CONTROL_STOP) ||
       (pDecisionAO->can_enable == CAN_DISABLE) ||
       (pose_warmup_cycles < GIMBAL_OUTPUT_GUARD_CYCLES)){
        gimbal_target_yaw_d = gimbal_yaw_d;
        reset_pitch_controller();
        reset_yaw_controller();
    }

#if defined GIMBAL_OFF
    pitch_target_output = 0;
#endif
}

void GimbalEstimateUpdate(void){
}

void GimbalSetPitchTarget(float pitch_target_d){
    gimbal_target_pitch_d = pitch_target_d;
}

void GimbalSetTarget(float yaw_target_d, float pitch_target_d){
    gimbal_target_yaw_d = yaw_target_d;
    gimbal_target_pitch_d = pitch_target_d;
}

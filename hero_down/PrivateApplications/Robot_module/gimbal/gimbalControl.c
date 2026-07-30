/**
 * @file    gimbalControl.c
 * @brief   云台控制模块 —— yaw轴输入决策、观测估计、闭环控制
 * @note    从 robot_control_task 拆分，集中管理云台所有控制逻辑
 */

#include <math.h>

#include "gimbalControl_internal.h"

#include "../../PrivateDrivers/Board2Borad/Board2Board.h"
#include "general_define.h"
#include "task_receive.h"
#include "task_state.h"
#include "stirControl.h"
#include "decision_ao.h"

static gimbal_runtime_t g_gimbal_runtime = {0};
const GimbalControl *const _gimbalControl = &g_gimbal_runtime.control;

/*---------------------------------------------------------------------------模块级变量-----------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------初始化-------------------------------------------------------------------------------------------*/

/**
 * @brief 云台控制初始化（yaw轴PID / LTD / TD）
 * @note  由 ControlInit() 调用
 */
void GimbalDecisionInitialize(void){
    SmoothFilterInitialize(&mouse_filter_x, GIMBAL_MOUSE_FILTER_ALPHA);
    SmoothFilterInitialize(&mouse_filter_y, GIMBAL_MOUSE_FILTER_ALPHA);
}

void GimbalControlInitialize(void){
}

void GimbalTargetAlignToEstimate(void){
    gimbal_target_yaw_d = gimbal_yaw_d;
    gimbal_target_pitch_d = gimbal_pitch_d;
}
/*---------------------------------------------------------------------------输入决策更新-------------------------------------------------------------------------------------*/

static void gimbal_update_remote_input(void){
    if(!gimbal_yaw_rx_valid){
        GimbalTargetAlignToEstimate();
        return;
    }

    if(pDecisionAO->sniper == SNIPER_ON){
        float pitch_delta_d;

        if(pDecisionAO->world_enable == WORLD_ENABLE_ON){
            gimbal_target_yaw_d = gimbal_yaw_target_rx_d;
        } else {
            gimbal_target_yaw_d += gimbal_rc_yaw * GIMBAL_SNIPER_YAW_GAIN_D -
                                   gimbal_rc_yaw * GIMBAL_SEPARATE_INPUT_SCALE *
                                       (pDecisionAO->chassis_mode == CHASSIS_SEPARATE);
        }

        pitch_delta_d =
            GIMBAL_SNIPER_PITCH_GAIN_D * (gimbal_rc_pitch * GIMBAL_REMOTE_PITCH_SCALE -
                                          gimbal_rc_pitch * GIMBAL_SEPARATE_INPUT_SCALE *
                                              (pDecisionAO->chassis_mode == CHASSIS_SEPARATE));
        gimbal_target_pitch_d += pitch_delta_d;
    }

    if(pDecisionAO->sniper == SNIPER_OFF){
        gimbal_target_yaw_d += gimbal_rc_yaw * GIMBAL_NORMAL_YAW_GAIN_D -
                               gimbal_rc_yaw * GIMBAL_SEPARATE_INPUT_SCALE *
                                   (pDecisionAO->chassis_mode == CHASSIS_SEPARATE);
        gimbal_target_pitch_d -=
            GIMBAL_NORMAL_PITCH_GAIN_D * (gimbal_rc_pitch * GIMBAL_REMOTE_PITCH_SCALE -
                                          gimbal_rc_pitch * GIMBAL_SEPARATE_INPUT_SCALE *
                                              (pDecisionAO->chassis_mode == CHASSIS_SEPARATE));
    }

#ifndef SHOOT_OFF
    ShootSetEnabled(ENABLE);
#else
    ShootSetEnabled(DISABLE);
#endif
}

static void gimbal_update_pc_sniper_trim(float *yaw_delta_d, float *pitch_delta_d){
    int yaw_key_direction;
    int pitch_key_direction;

    yaw_key_direction = gimbal_yaw_key_direction;
    if(yaw_key_direction != GIMBAL_KEY_RELEASED){
        last_yaw_key_direction = (int8_t)yaw_key_direction;
        yaw_key_hold_cycles++;
        if(yaw_key_hold_cycles > GIMBAL_KEY_HOLD_THRESHOLD_CYCLES){
            *yaw_delta_d = yaw_key_direction * GIMBAL_KEY_HOLD_STEP_D;
            yaw_trim_d += yaw_key_direction * GIMBAL_KEY_HOLD_STEP_D;
        }
    }
    if(yaw_key_direction == GIMBAL_KEY_RELEASED && last_yaw_key_direction != GIMBAL_KEY_RELEASED){
        if(yaw_key_hold_cycles <= GIMBAL_KEY_HOLD_THRESHOLD_CYCLES){
            yaw_trim_d += last_yaw_key_direction * GIMBAL_KEY_TAP_STEP_D;
            *yaw_delta_d += last_yaw_key_direction * GIMBAL_KEY_TAP_STEP_D;
        }
        last_yaw_key_direction = GIMBAL_KEY_RELEASED;
        yaw_key_hold_cycles = 0U;
    }

    pitch_key_direction = gimbal_pitch_key_direction;
    if(pitch_key_direction != GIMBAL_KEY_RELEASED){
        last_pitch_key_direction = (int8_t)pitch_key_direction;
        pitch_key_hold_cycles++;
        if(pitch_key_hold_cycles > GIMBAL_KEY_HOLD_THRESHOLD_CYCLES){
            *pitch_delta_d = pitch_key_direction * GIMBAL_KEY_HOLD_STEP_D;
            pitch_trim_d += pitch_key_direction * GIMBAL_KEY_HOLD_STEP_D;
        }
    }
    if(pitch_key_direction == GIMBAL_KEY_RELEASED &&
       last_pitch_key_direction != GIMBAL_KEY_RELEASED){
        if(pitch_key_hold_cycles <= GIMBAL_KEY_HOLD_THRESHOLD_CYCLES){
            pitch_trim_d += last_pitch_key_direction * GIMBAL_KEY_TAP_STEP_D;
            *pitch_delta_d += last_pitch_key_direction * GIMBAL_KEY_TAP_STEP_D;
        }
        last_pitch_key_direction = GIMBAL_KEY_RELEASED;
        pitch_key_hold_cycles = 0U;
    }
}

static void gimbal_update_pc_input(void){
    float yaw_delta_d = 0.0f;
    float pitch_delta_d = 0.0f;
    float mouse_filter_gain = GIMBAL_MOUSE_GAIN_NORMAL;

    if(_robotState->aim_mode && pDecisionAO->sniper == SNIPER_ON){
        gimbal_target_yaw_d = gimbal_yaw_target_rx_d;
    } else {
        if(pDecisionAO->sniper == SNIPER_ON){
            mouse_filter_gain = GIMBAL_MOUSE_GAIN_SNIPER;
            gimbal_update_pc_sniper_trim(&yaw_delta_d, &pitch_delta_d);
        }

        if(pDecisionAO->sniper == SNIPER_OFF){
            yaw_delta_d +=
                SmoothFilterUpdate(&mouse_filter_x, gimbal_mouse_speed_x) * mouse_filter_gain;
            pitch_delta_d +=
                SmoothFilterUpdate(&mouse_filter_y, gimbal_mouse_speed_y) * mouse_filter_gain;
            yaw_trim_d = 0.0f;
            pitch_trim_d = 0.0f;
        }

        if(pDecisionAO->sniper != SNIPER_ON){
            if(!gimbal_yaw_rx_valid && pDecisionAO->sniper == SNIPER_OFF){
                GimbalTargetAlignToEstimate();
            } else {
                gimbal_target_yaw_d += yaw_delta_d + yaw_trim_d;
                gimbal_target_pitch_d += pitch_delta_d + pitch_trim_d;
            }
        }
    }

#ifndef GIMBAL_OFF
    ShootSetEnabled(ENABLE);
#else
    ShootSetEnabled(DISABLE);
#endif
}

static void gimbal_update_fixed_pitch(void){
    if(pDecisionAO->sniper == SNIPER_ON){
        if(gimbal_rc_fixed_pitch > SNIPER_PITCH_FIXED_CH1_THRESHOLD){
            pitch_fixed_mode = GIMBAL_PITCH_FIXED_ENABLED;
        } else if(gimbal_rc_fixed_pitch < -SNIPER_PITCH_FIXED_CH1_THRESHOLD){
            pitch_fixed_mode = GIMBAL_PITCH_FIXED_DISABLED;
        }

        if(pitch_fixed_mode == GIMBAL_PITCH_FIXED_ENABLED){
            gimbal_target_pitch_d = SNIPER_PITCH_FIXED_DEG;
        }
    } else {
        pitch_fixed_mode = GIMBAL_PITCH_FIXED_DISABLED;
    }
}

/**
 * @brief 在下板统一计算 yaw/pitch 最终目标
 */
void GimbalInputUpdate(void){
    switch(pDecisionAO->ctrl_terminal){
        case CONTROL_STOP:
            GimbalTargetAlignToEstimate();
            break;

        case CONTROL_FROM_REMOTE:
            gimbal_update_remote_input();
            break;

        case CONTROL_FROM_PC:
            gimbal_update_pc_input();
            break;
    }

    gimbal_update_fixed_pitch();
    gimbal_target_yaw_d = AngleLimit(gimbal_target_yaw_d, -GIMBAL_HALF_TURN_D, GIMBAL_HALF_TURN_D);
}

/*---------------------------------------------------------------------------观测更新-----------------------------------------------------------------------------------------*/

/**
 * @brief 云台相关观测数据更新
 */
void GimbalEstimateUpdate(void){
    /* 当前由 GimbalPoseUpdate() 在外层 IMU 任务中更新，此处预留。 */
}

void GimbalPoseUpdate(float pitch_angle_d, float pitch_angular_velocity_dps, float yaw_angle_d,
                      float yaw_angular_velocity_dps, float roll_angle_d,
                      float roll_angular_velocity_dps){
    (void)pitch_angle_d;
    (void)pitch_angular_velocity_dps;
    (void)yaw_angle_d;
    (void)yaw_angular_velocity_dps;

    gimbal_roll_d = roll_angle_d;
    gimbal_roll_dps = roll_angular_velocity_dps;

    /* yaw 编码器已迁到上板，位姿由 B2B 接收。 */
    B2B_PoseAliveTick();

    if(isfinite(gimbal_yaw_rx_d)){
        gimbal_yaw_d = gimbal_yaw_rx_d;
    }
    if(isfinite(gimbal_yaw_dps_rx)){
        gimbal_yaw_dps = gimbal_yaw_dps_rx;
    }
    if(isfinite(gimbal_pitch_rx_d)){
        gimbal_pitch_d = gimbal_pitch_rx_d;
    }
    if(isfinite(gimbal_pitch_dps_rx)){
        gimbal_pitch_dps = gimbal_pitch_dps_rx;
    }

    if(target_align_delay_cycles < GIMBAL_POSE_WARMUP_CYCLES){
        target_align_delay_cycles++;
    }

    if(previous_sniper_mode != pDecisionAO->sniper){
        target_align_delay_cycles = 0U;
        gimbal_target_yaw_d = gimbal_yaw_d;
        gimbal_target_pitch_d = gimbal_pitch_d;
        previous_sniper_mode = pDecisionAO->sniper;
    }
}

/*---------------------------------------------------------------------------闭环控制-----------------------------------------------------------------------------------------*/

/**
 * @brief 云台yaw轴闭环控制（MIT力矩/速度指令输出）
 */
void GimbalControlUpdate(void){
    if(last_ctrl_terminal == CONTROL_STOP && pDecisionAO->ctrl_terminal != CONTROL_STOP){
        gimbal_target_yaw_d = gimbal_yaw_d;
        gimbal_target_pitch_d = gimbal_pitch_d;
        target_align_delay_cycles = 0U;
    }
    last_ctrl_terminal = pDecisionAO->ctrl_terminal;

    if(CONTROL_STOP == pDecisionAO->ctrl_terminal ||
       target_align_delay_cycles < GIMBAL_TARGET_GUARD_CYCLES){
        gimbal_target_yaw_d = gimbal_yaw_d;
        gimbal_target_pitch_d = gimbal_pitch_d;
    }
}

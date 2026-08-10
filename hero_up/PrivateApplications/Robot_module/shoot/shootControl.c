/**
 * @file shootControl.c
 * @brief 摩擦轮目标生成和速度闭环控制
 */

#include "shootControl_internal.h"

#include <string.h>

#include "decision_ao.h"
#include "task_receive.h"

static shoot_runtime_t g_shoot_runtime = {0};
const ShootControl *const _shootControl = &g_shoot_runtime.control;

extern uint8_t shit_dan;

static void set_fric_targets(float left_rpm, float right_rpm, float up_rpm,
                             float left_1_rpm, float right_1_rpm, float up_1_rpm);

static void set_fric_targets(float left_rpm, float right_rpm, float up_rpm,
                             float left_1_rpm, float right_1_rpm, float up_1_rpm){
    fric_target_speed_rpm[LEFT] = left_rpm;
    fric_target_speed_rpm[RIGHT] = right_rpm;
    fric_target_speed_rpm[UP] = up_rpm;
    fric_target_speed_rpm[LEFT1] = left_1_rpm;
    fric_target_speed_rpm[RIGHT1] = right_1_rpm;
    fric_target_speed_rpm[UP1] = up_1_rpm;
}

void ShootControlInitialize(void){
    memset(&g_shoot_runtime, 0, sizeof(g_shoot_runtime));

    for(uint8_t motor_index = 0U; motor_index < SHOOT_FRIC_MOTOR_COUNT; motor_index++){
        PIDInitialize(&fric_speed_pid[motor_index],
                      SHOOT_FRIC_PID_KP,
                      SHOOT_FRIC_PID_KI,
                      SHOOT_FRIC_PID_KD,
                      SHOOT_FRIC_PID_INTEGRAL_LIMIT,
                      TEMP_SHOOT_3508_CURRENT_MAX);
    }
}

void ShootInputUpdate(void){
    uint8_t can_enable = pDecisionAO->can_enable;
    uint8_t ctrl_terminal = pDecisionAO->ctrl_terminal;
    uint8_t fric_mode = pDecisionAO->fric_mode;
    uint8_t sniper = pDecisionAO->sniper;

    if(shit_dan){
        BulletSpeedReceive();
        shit_dan = 0U;
    }

    if((ctrl_terminal == CONTROL_STOP) || (can_enable == CAN_DISABLE)){
        set_fric_targets(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    } else if(fric_mode == FRIC_ON){
        if(sniper == SNIPER_ON){
            set_fric_targets(SHOOT_FRIC_FRONT_SPEED_RPM,
                             -SHOOT_FRIC_FRONT_SPEED_RPM,
                             SHOOT_FRIC_BACK_SPEED_RPM,
                             SHOOT_FRIC_BACK_SPEED_RPM,
                             -SHOOT_FRIC_BACK_SPEED_RPM,
                             SHOOT_FRIC_FRONT_SPEED_RPM);
        } else {
            set_fric_targets(SHOOT_FRIC_DEFAULT_SPEED_RPM,
                             -SHOOT_FRIC_DEFAULT_SPEED_RPM,
                             SHOOT_FRIC_DEFAULT_SPEED_RPM,
                             SHOOT_FRIC_DEFAULT_SPEED_RPM,
                             -SHOOT_FRIC_DEFAULT_SPEED_RPM,
                             SHOOT_FRIC_DEFAULT_SPEED_RPM);
        }
    } else {
        set_fric_targets(-SHOOT_FRIC_IDLE_SPEED_RPM,
                         SHOOT_FRIC_IDLE_SPEED_RPM,
                         -SHOOT_FRIC_IDLE_SPEED_RPM,
                         -SHOOT_FRIC_IDLE_SPEED_RPM,
                         SHOOT_FRIC_IDLE_SPEED_RPM,
                         -SHOOT_FRIC_IDLE_SPEED_RPM);
    }
}

void ShootEstimateUpdate(void){
}

void ShootControlUpdate(void){
    for(uint8_t motor_index = 0U; motor_index < SHOOT_FRIC_MOTOR_COUNT; motor_index++){
        fric_target_output[motor_index] =
            PIDUpdate(&fric_speed_pid[motor_index],
                      fric_target_speed_rpm[motor_index] -
                          _fricMotorRec[motor_index].mechanical_speed_rpm);
    }
}

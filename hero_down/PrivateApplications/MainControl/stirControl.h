#ifndef _STIR_CONTROL_H_
#define _STIR_CONTROL_H_

#include <stdint.h>
#include "pid.h"
/* ShootControl 类型定义 + 射击相关常量 — 从 robot_control_task.h 搬迁 */

#define STIR_PRESET_ANGLE +35.0f
#define STIR_CAUTION_SPEED 1.0f
#define STIR_MAX_SPEED      300
#define STIR_MAX_SPEED_LOW  300
#define SHOOT_FINISH 0
#define SHOOT_BACK   1
#define SHOOT_READY  2
#define SHOOT_PUSH   3
#define SHOOT_WAIT   5
#define SHOOT_DURING 4
#ifdef MATCH_MODE
#define INITIAL_FRIC_SPEED 4700
#else
#define INITIAL_FRIC_SPEED 4700
#endif
#define TARGET_BULLET_SPEED 15.10

/**
 * @brief 发射控制相关结构体
 */
typedef struct
{
    struct
    {
        float fric_speed_rpm[6];
        float stir_speed_rps;
        float stir_angle_d;
        float stir_target_pos;
        float stir_target_pos_rad;
        float stir_target_vol;
        float stir_all_target_pos_d;
        float stir_all_target_pos_rad;
        int shoot_flag;
        int shoot_cnt;
    }ShootTargetInput;

    struct
    {
        PIDStruct fric_speed_pid[6];
        int16_t fric_target_output[6];
        float stir_preset_angle;
        float stir_angular_velocity_dps;
    }ShootMotorControl;

    struct
    {
        uint8_t stir_block_flag;
        uint8_t stir_reset_flag;
        uint8_t stir_enableflag_detect;
        uint8_t stir_enableflag_desire;
        uint16_t shoot_count;
        float stir_real_angle;
        float stir_real_angle_rad;
        float stir_real_angle_d;
        float stir_angle_last;
        float stir_angle_cur;
        float stir_all_angle_d;
        int quan_shu_r;
    }ShootEstimate;
}ShootControl;

/* 全局实例 */
extern ShootControl shootControl;

/* 模块级变量（跨 task_decision / task_estimate / task_control 共用） */
extern uint16_t stall_count;
extern uint8_t  stir_stall_recovery_state;
extern uint8_t  stir_flag;
extern uint8_t  stir_two_step_phase;
extern uint16_t stir_delay_counter;

/* 射击/摩擦轮变量 */
extern float predict_speed0;
extern float mardio_speed;
extern float current_fric_speed;

/* 输入决策（DecisionTask 调用） */
void ShootInputUpdate(void);

/* 观测估计（EstimateTask 调用） */
void ShootEstimateUpdate(void);

/* 闭环控制（ControlTask 调用） */
void ShootControlUpdate(void);

/* 拨盘辅助函数 */
void GetStirRealAngle(void);
void StirTargetAngleSet(void);
uint16_t CRC16_Modbus(uint8_t *data, uint16_t length);

#endif

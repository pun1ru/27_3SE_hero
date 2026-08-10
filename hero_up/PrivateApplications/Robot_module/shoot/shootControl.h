#ifndef SHOOT_CONTROL_H_
#define SHOOT_CONTROL_H_

#include <stdint.h>

#include "pid.h"

#define SHOOT_FRIC_MOTOR_COUNT 6U

/**
 * @brief 发射控制状态
 */
typedef struct {
    struct {
        float fric_speed_rpm[SHOOT_FRIC_MOTOR_COUNT];
        float stir_speed_rps;
        float stir_angle_d;
        float stir_target_pos;
        float stir_target_pos_rad;
        float stir_target_vol;
        float stir_all_target_pos_d;
        float stir_all_target_pos_rad;
        int shoot_flag;
        int shoot_cnt;
    } ShootTargetInput;

    struct {
        PIDStruct fric_speed_pid[SHOOT_FRIC_MOTOR_COUNT];
        int16_t fric_target_output[SHOOT_FRIC_MOTOR_COUNT];
        float stir_preset_angle;
        float stir_angular_velocity_dps;
    } ShootMotorControl;

    struct {
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
    } ShootEstimate;
} ShootControl;

extern const ShootControl *const _shootControl;

/**
 * @brief   初始化发射控制器和运行状态
 * @param   void
 * @retval  void
 */
void ShootControlInitialize(void);

/**
 * @brief   更新发射目标输入
 * @param   void
 * @retval  void
 */
void ShootInputUpdate(void);

/**
 * @brief   更新发射估计阶段
 * @param   void
 * @retval  void
 */
void ShootEstimateUpdate(void);

/**
 * @brief   更新发射闭环控制
 * @param   void
 * @retval  void
 */
void ShootControlUpdate(void);

#endif

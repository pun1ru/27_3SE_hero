#ifndef _CHASSIS_CONTROL_H_
#define _CHASSIS_CONTROL_H_

#include <stdint.h>
#include "pid.h"
/* ChassisControl 类型定义 — 从 robot_control_task.h 搬迁 */

#define CHASSIS_MOTOR_FEEDFORWARD_OUTPUT_PER_MPS 1500.0f

/**
 * @brief 底盘控制相关结构体，存放闭环控制器，目标赋值等等
 */
typedef struct {
    /*底盘跟随控制相关*/
    struct {
        PIDStruct follow_speed_need_pid;
        int8_t revolve_return_flag;
    } ChassisFollowControl;

    /*云台坐标系下的目标速度*/
    struct {
        float speed_x_mps;
        float speed_y_mps;
        float max_revolve_speed_rps;
        PIDStruct speed_x_compensate_pid;
        PIDStruct speed_y_compensate_pid;
    } GimbalCoordinateInput;

    /*底盘坐标系下的目标速度*/
    struct {
        float speed_x_mps;
        float speed_y_mps;
        float speed_w_rps;
        float compensate_speed_w_dps;
    } ChassisCoordinateInput;

    /*底盘实际需要的目标速度*/
    struct {
        float speed_x_mps;
        float speed_y_mps;
        float speed_w_rps;
        float power_limit_scale;
        float compensate_power;
    } ChassisRealNeedInput;

    /*轮电机的目标转速及控制*/
    struct {
        float target_speed_mps[4];
        int16_t target_motor_output[4];
        PIDStruct speed_control_pid[4];
    } WheelMotorControl;

    /*底盘观测真实数据*/
    struct {
        float gimbal_to_chassis_delta_angle_d;
        float chassis_follow_angle_d;
        float wheel_real_speed_mps[4];
        float speed_x_mps;
        float speed_y_mps;
        float speed_w_rps;
        float imu_yaw_dps;
    } ChassisEstimate;

    struct {
        int16_t total_output_power;
        int16_t state;
        float max_compensate_power;
    } SuperCapacity;
} ChassisControl;

/* 全局实例 */
extern const ChassisControl *const _chassisControl;

/**
 * @brief   初始化 Decision 阶段使用的底盘控制器
 * @retval  void
 */
void ChassisDecisionInitialize(void);

/**
 * @brief   初始化 Control 阶段使用的底盘控制器
 * @retval  void
 */
void ChassisControlInitialize(void);

/* 输入决策（DecisionTask 调用） */
void ChassisInputUpdate(void);

/* 观测估计（EstimateTask 调用） */
void ChassisEstimateUpdate(void);

/* 闭环控制（ControlTask 调用） */
void ChassisControlUpdate(void);

#endif

#ifndef _ROBOT_CONTROL_TASK_H_
#define _ROBOT_CONTROL_TASK_H_
#include "stdint.h"

#include "pid.h"
#include "algorism.h"
#include "adrc.h"
#include "DMJ4310.h"

/* MainControl 模块类型定义 */
#include "gimbalControl.h"
#include "shootControl.h"
#include "worldGimbal.h"

/* ChassisControl 类型 — hero_up 保留用于 B2B 通信（底盘由 hero_down 控制） */
typedef struct
{
    struct
    {
        PIDStruct follow_speed_need_pid;
        int8_t revolve_return_flag;
    }ChassisFollowControl;
    struct
    {
        float speed_x_mps;
        float speed_y_mps;
        float max_revolve_speed_rps;
        PIDStruct speed_x_compensate_pid;
        PIDStruct speed_y_compensate_pid;
    }GimbalCoordinateInput;
    struct
    {
        float speed_x_mps;
        float speed_y_mps;
        float speed_w_rps;
        float compensate_speed_w_dps;
    }ChassisCoordinateInput;
    struct
    {
        float speed_x_mps;
        float speed_y_mps;
        float speed_w_rps;
        float power_limit_scale;
        float compensate_power;
    }ChassisRealNeedInput;
    struct
    {
        float target_speed_mps[4];
        int16_t target_motor_output[4];
        PIDStruct speed_control_pid[4];
    }WheelMotorControl;
    struct
    {
        float gimbal_to_chassis_delta_angle_d;
        float chassis_follow_angle_d;
        float wheel_real_speed_mps[4];
        float speed_x_mps;
        float speed_y_mps;
        float speed_w_rps;
        float imu_yaw_dps;
    }ChassisEstimate;
    struct
    {
        int16_t total_output_power;
        int16_t state;
        float max_compensate_power;
    }SuperCapacity;
}ChassisControl;

#endif

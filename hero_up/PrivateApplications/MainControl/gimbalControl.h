#ifndef _GIMBAL_CONTROL_H_
#define _GIMBAL_CONTROL_H_

#include "stdint.h"
#include "pid.h"
#include "algorism.h"
#include "adrc.h"
#include "DMJ4310.h"

#define PITCH_OFFSET_MACHENICAL_ANGLE (58267-1487-1220)

/**
 * @brief 云台控制相关结构体
 */
typedef struct
{
    /*云台目标输入*/
    struct
    {
        float pitch_angle_d;
        float small_pitch_angle_d;
        float yaw_angle_d;
        float pitch_angular_velocity_dps;
        float yaw_angular_velocity_dps;
        float yaw_recoil_compensation_d;
    }GimbalTargetInput;
    /*云台控制相关*/
    struct
    {
        PIDStruct pitch_calibration_pid;
        ADRC pitch_angle_adrc;
        PIDStruct pitch_speed_pid;
        LTD pitch_LTD;
        LTDPID pitch_LTD_pid;
        int16_t pitch_target_output;
        int16_t small_pitch_target_output;
        uint32_t sniper_pos;
        uint16_t sniper_max_speed;
        uint8_t spin_dir;
    }GimbalMotorControl;
    /*云台真实姿态观测*/
    struct
    {
        float pitch_angle_d;
        float pitch_angle_before;
        float pitch_angular_velocity_dps;
        float small_pitch_actual_angle;
        float yaw_angle_d;
        float yaw_angular_velocity_dps;
        float roll_angle_d;
        float roll_angular_velocity_dps;
    }GimbalEstimate;
}GimbalControl;

/* API */
void GimbalInit(void);
void GimbalInputUpdate(void);
void GimbalEstimateUpdate(void);
void GimbalControlUpdate(void);
void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w, float roll_angle, float roll_angle_w);

extern GimbalControl gimbalControl;
extern const GimbalControl* _gimbalControl;
extern SmoothFilter MouseFilterX;
extern SmoothFilter MouseFilterY;
extern float pitch_angle_from_match;
extern float kpfric;
extern float kdfric;

#endif

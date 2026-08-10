#ifndef GIMBAL_CONTROL_H_
#define GIMBAL_CONTROL_H_

#include <stdint.h>

#include "adrc.h"
#include "algorism.h"
#include "pid.h"

#define PITCH_OFFSET_MACHENICAL_ANGLE 56183

#define PITCH_ESO_Z1_MIN_D (-15.0f)
#define PITCH_ESO_Z1_MAX_D 36.0f
#define PITCH_ESO_Z2_MIN_DPS (-200.0f)
#define PITCH_ESO_Z2_MAX_DPS 200.0f

/**
 * @brief 云台控制状态
 */
typedef struct {
    struct {
        float pitch_angle_d;
        float yaw_angle_d;
        float pitch_angular_velocity_dps;
        float yaw_angular_velocity_dps;
    } GimbalTargetInput;

    struct {
        PIDStruct pitch_calibration_pid;
        ADRC pitch_angle_adrc;
        PIDStruct pitch_speed_pid;
        LTD pitch_LTD;
        LTDPID pitch_LTD_pid;
        int16_t pitch_target_output;

        ADRC yaw_ADRC;
        LTD yaw_LTD;
        PIDStruct yaw_pos_pid;
        PIDStruct yaw_speed_pid;
        float yaw_target_output;
        float w_d;
        float pre_yaw_Tff;

        uint32_t sniper_pos;
        uint16_t sniper_max_speed;
        uint8_t spin_dir;
    } GimbalMotorControl;

    struct {
        float pitch_angle_d;
        float pitch_angle_before;
        float pitch_angular_velocity_dps;
        float yaw_angle_d;
        float yaw_angular_velocity_dps;
        float roll_angle_d;
        float roll_angular_velocity_dps;
    } GimbalEstimate;
} GimbalControl;

extern const GimbalControl *const _gimbalControl;
extern const float yaw_dm_forward_offset_rad;

/**
 * @brief   初始化云台控制器和运行状态
 * @param   void
 * @retval  void
 */
void GimbalControlInitialize(void);

/**
 * @brief   更新云台目标输入
 * @param   void
 * @retval  void
 */
void GimbalInputUpdate(void);

/**
 * @brief   更新云台估计阶段
 * @param   void
 * @retval  void
 */
void GimbalEstimateUpdate(void);

/**
 * @brief   更新云台闭环控制
 * @param   void
 * @retval  void
 */
void GimbalControlUpdate(void);

/**
 * @brief   更新云台姿态观测
 * @param   pitch_angle_d pitch 角度，单位 deg
 * @param   pitch_angular_velocity_radps pitch 角速度，单位 rad/s
 * @param   yaw_angle_d yaw 角度，单位 deg
 * @param   yaw_angular_velocity_radps yaw 角速度，单位 rad/s
 * @param   roll_angle_d roll 角度，单位 deg
 * @param   roll_angular_velocity_dps roll 角速度，单位 deg/s
 * @retval  void
 */
void GimbalPoseUpdate(float pitch_angle_d, float pitch_angular_velocity_radps,
                      float yaw_angle_d, float yaw_angular_velocity_radps,
                      float roll_angle_d, float roll_angular_velocity_dps);

/**
 * @brief   设置 pitch 目标角
 * @param   pitch_target_d pitch 目标角，单位 deg
 * @retval  void
 */
void GimbalSetPitchTarget(float pitch_target_d);

/**
 * @brief   设置 yaw 与 pitch 目标角
 * @param   yaw_target_d yaw 目标角，单位 deg
 * @param   pitch_target_d pitch 目标角，单位 deg
 * @retval  void
 */
void GimbalSetTarget(float yaw_target_d, float pitch_target_d);

#endif

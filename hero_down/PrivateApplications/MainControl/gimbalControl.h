#ifndef _GIMBAL_CONTROL_H_
#define _GIMBAL_CONTROL_H_

#include "pid.h"
#include "adrc.h"
#include "DMJ4310.h"

/**
 * @brief 云台控制相关结构体
 */
typedef struct
{
	/*云台目标输入*/
	struct
	{
		float yaw_angle_d;		//期望的yaw目标角（DecisionTask设置，B2B 0x220发送）
		float yaw_angular_velocity_dps; //yaw期望角速度
		float pitch_angle_d;	//期望的pitch目标角（DecisionTask设置，B2B 0x221发送）
	}GimbalTargetInput;
	/*云台控制相关（yaw控制已搬迁至上板）*/
	struct
	{
		uint32_t sniper_pos;
		uint16_t sniper_max_speed;
		uint8_t spin_dir;
	}GimbalMotorControl;

	/*云台真实姿态观测*/
	struct
	{
		float pitch_angle_d;
		float pitch_angular_velocity_dps;
		float small_pitch_actual_angle;
		float yaw_angle_d;
		float yaw_angular_velocity_dps;
		float shoot_window_flag; // 发射窗口标志位: 1.0f窗口内, 0.0f窗口外

		float roll_angle_d;
		float roll_angular_velocity_dps;

	}GimbalEstimate;
}GimbalControl;

/* 全局云台控制实例 */
extern GimbalControl gimbalControl;
extern const GimbalControl* _gimbalControl;

/* 延时计数器，GimbalPoseUpdate 使用 */
extern uint8_t shit_delay_count;

/* 云台初始化（yaw轴PID/LTD/TD初始化），由 ControlInit 调用 */
void GimbalInit(void);

/* 云台yaw输入决策更新，由 DecisionTask 调用 */
void GimbalInputUpdate(void);

/* 云台观测数据更新，由 ControlTask 调用 */
void GimbalEstimateUpdate(void);

/* 云台闭环控制更新，由 ControlTask 调用 */
void GimbalControlUpdate(void);

/* 云台位姿观测更新，由 IMU 任务调用 */
void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w, float roll_angle, float roll_angle_w);

#endif

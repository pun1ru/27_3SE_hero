#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H

#include <stdint.h>

#include "arm_matrix.h"

/* 关节状态定义 */
#define JOINT_STOP   -1
#define JOINT_NORMAL  0
#define JOINT_CLIMB   1

#define JOINT_STAND_MODE_NORMAL     0
#define JOINT_STAND_MODE_PRE_STAIR  1
#define JOINT_STAND_MODE_STAIR_UP   2
#define JOINT_STAND_MODE_PRE_DOWN_STAIR 3

#define JOINT_JUMP_MODE_OFF 0
#define JOINT_JUMP_MODE_ON  1

/* 关节力控模块维度定义 */
#define JOINT_CTRL_LEG_NUM   4
#define JOINT_CTRL_AXIS_NUM  3
#define JOINT_CTRL_MOTOR_NUM 4

typedef enum
{
	LEG_LF = 0,
	LEG_RF = 1,
	LEG_RB = 2,
	LEG_LB = 3,
} JointLegId;

/**
 * @brief 关节控制相关结构体（从 robot_control_task.h 迁移而来）
 */
typedef struct
{
	/* 关节电机控制相关 */
	struct
	{
		float mit_p[4];
		float mit_v[4];
		float mit_Kp[4];
		float mit_Kd[4];
		float mit_Tff[4];
		int16_t target_output[4];
		uint8_t enable_flag[4];
	} JointMotorControl;

	/* 关节电机回传估计 */
	struct
	{
		uint16_t frame_counter[4];
		uint32_t id[4];
		int state[4];
		float pos_d[4];
		float motor_angles_rad[4];
		float vel_radps[4];
		float toq[4];
		float Kp[4];
		float Kd[4];
		float Tmos[4];
		float Tcoil[4];
		float body_height_m;
		float body_height_vel_mps;
	} JointEstimate;

	int joint_state;
} JointControl;

/* 机体状态（由 IMU / 状态估计提供） */
typedef struct
{
	float roll_d;
	float pitch_d;
	float yaw_d;
	float roll_rate_dps;
	float pitch_rate_dps;
	float yaw_rate_dps;
	float accel_x;  /* 加速度 x，单位 m/s² */
	float accel_y;  /* 加速度 y，单位 m/s² */
	float accel_z;  /* 加速度 z，单位 m/s² */
} JointBodyState;

/* 机体目标（上层控制给定） */
typedef struct
{
	float roll_d;
	float pitch_d;
	float yaw_d;
	float roll_rate_dps;
	float pitch_rate_dps;
	float yaw_rate_dps;
	float heave_m;
	float heave_vel_mps;
} JointBodyTarget;

/* 机体广义力需求：合力 + 合力矩 */
typedef struct
{
	float force_b[JOINT_CTRL_AXIS_NUM];
	float torque_b[JOINT_CTRL_AXIS_NUM];
} JointBodyWrenchDemand;

/* 力控参数与中间量 */
// //kp_att[3] kd_att[3] 姿态 PD 增益。
// kp_heave kd_heave 高度（悬架）PD 增益。
// force_limit_n[4][3] 每条腿每个方向的力限幅。
// contact_flag[4] 接触状态，1 表示承重腿，0 表示悬空腿。
// leg_pos_body_m[4][3] 腿端相对机体质心位置，用于力矩分配。
// leg_force_cmd_b_n[4][3] 每条腿在机体系下的目标接触力。
// leg_force_lpf_n[4][3] 每条腿接触力低通结果，抑制高频振动。
// motor_torque_ref_nm[4] 每电机参考扭矩。
//wrench_demand 当前周期机体广义力目标缓存
typedef struct
{
	float body_mass_kg;
	float gravity_mps2;
	float dt_s;

	float kp_att[JOINT_CTRL_AXIS_NUM];
	float kd_att[JOINT_CTRL_AXIS_NUM];

	float kp_heave;
	float kd_heave;

	float force_limit_n[JOINT_CTRL_LEG_NUM][JOINT_CTRL_AXIS_NUM];
	uint8_t contact_flag[JOINT_CTRL_LEG_NUM];

	float leg_pos_body_m[JOINT_CTRL_LEG_NUM][JOINT_CTRL_AXIS_NUM];
	float leg_force_cmd_b_n[JOINT_CTRL_LEG_NUM][JOINT_CTRL_AXIS_NUM];
	float leg_force_lpf_n[JOINT_CTRL_LEG_NUM][JOINT_CTRL_AXIS_NUM];
	float motor_torque_ref_nm[JOINT_CTRL_MOTOR_NUM];

	JointBodyWrenchDemand wrench_demand;
} JointForceControlContext;

extern matrix_data_t g_joint_rot_leg_to_body_data[9];
extern matrix_data_t g_joint_rot_body_to_imu_data[9];
extern matrix_data_t g_joint_rot_imu_to_world_data[9];

extern matrix_t g_joint_rot_leg_to_body;
extern matrix_t g_joint_rot_body_to_imu;
extern matrix_t g_joint_rot_imu_to_world;

/* 每条腿的位移对关节角偏导（3x1） */
extern matrix_data_t g_joint_leg_jacobian_dpos_dtheta_data[JOINT_CTRL_LEG_NUM][3];
extern matrix_t g_joint_leg_jacobian_dpos_dtheta[JOINT_CTRL_LEG_NUM];

extern float g_joint_motor_torque_cmd_nm[JOINT_CTRL_MOTOR_NUM];
extern float g_joint_motor_torque_fdb_nm[JOINT_CTRL_MOTOR_NUM];
extern float g_joint_control_dt_s;
extern uint8_t g_joint_control_enable;
extern int g_joint_state;
extern JointForceControlContext g_joint_force_ctrl;
extern JointBodyState g_joint_body_state_body_dbg;

void JointControlMatrixInit(void);
void JointControlPlaceholdersReset(void);
void JointForceControlTuningParamInit(void);
void JointForceControlInit(float body_mass_kg, float dt_s);
void JointForceControlSetContact(const uint16_t frame_counter[JOINT_CTRL_LEG_NUM],
				 const float torque_fdb_nm[JOINT_CTRL_LEG_NUM],
				 uint8_t ctrl_active);
void JointBuildMotorAnglesRadFromFeedback(const float feedback_pos_rad[JOINT_CTRL_MOTOR_NUM],
					  float motor_angles_rad_out[JOINT_CTRL_MOTOR_NUM]);
void JointBuildFeedbackPosRadFromMotorAngles(const float motor_angles_rad_in[JOINT_CTRL_MOTOR_NUM],
					     float feedback_pos_rad_out[JOINT_CTRL_MOTOR_NUM]);
void JointGetMotorAngleLimitsRad(float angle_min_rad_out[JOINT_CTRL_MOTOR_NUM],
					 float angle_max_rad_out[JOINT_CTRL_MOTOR_NUM]);
void JointForceControlSetMotorAngleRad(const float motor_angle_rad[JOINT_CTRL_MOTOR_NUM]);
float JointGetClimbRearTargetAngleRad(void);
void JointUpdateLegPoseFromMotorAngle(void);
void JointUpdateLegJacobiansFromMotorAngle(void);
void JointEstimateBodyHeightVelocity(const JointBodyState* body_state,
				     const float joint_vel_radps[JOINT_CTRL_MOTOR_NUM],
				     float* body_height_m_out,
				     float* body_height_vel_mps_out);
void JointForceControlConvertBodyState(const JointBodyState* body_state_in,
				       JointBodyState* body_state_out);
void JointForceControlEstimateUpdate(void);
void JointForceControlSetJointMode(uint8_t joint_mode, float climb_pitch_hold_d);
void JointForceControlSetStandMode(uint8_t stand_mode);
void JointForceControlSetJumpMode(uint8_t jump_mode);
void JointForceControlStep(const JointBodyState* body_state,
						   const JointBodyTarget* body_target,
						   float body_height_m,
						   float body_height_vel_mps);

/* 上台阶检测：监测 distance > 290 连续 10 周期后降至 90 以下，置位 stair_up 模式 */
void JointStairUpDetect(void);
void JointStairUpDetectReset(void);
uint8_t JointStairUpIsDetected(void);

#endif

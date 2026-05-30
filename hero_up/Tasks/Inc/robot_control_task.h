#ifndef _ROBOT_CONTROL_TASK_H_
#define _ROBOT_CONTROL_TASK_H_
#include "stdint.h"

#include "pid.h"
#include "algorism.h"
#include "adrc.h"
#include "DMJ4310.h"

#define CHASSIS_FORWARD_YAW_MACHENICAL_ANGLE 5691 //底盘正向朝向时对应的yaw电机机械角    3832//5675
#define GIMBAL_FORWARD_YAW_MACHENICAL_ANGLE 5675//2240 //选定的云台正向朝向时的yaw电机机械角3832


#define PITCH_OFFSET_MACHENICAL_ANGLE (58267-1487-1220)	//p轴水平状态机械角1361 62964,6.7039
#define STIR_PRESET_ANGLE +10.0f				//拨盘预置角度
#define CHASSIS_MOTOR_FRONTFEED_RATIO 1500
#define STIR_CAUTION_SPEED 1.0f	//达妙拨盘无操作速度
#define STIR_MAX_SPEED 100		//60
#define STIR_MAX_SPEED_LOW 100
#define SHOOT_FINISH 0
#define SHOOT_BACK 1
#define SHOOT_READY 2
#define SHOOT_PUSH 3
#define SHOOT_WAIT 5
#define SHOOT_DURING 4

#ifdef MATCH_MODE
#define INITIAL_FRIC_SPEED 4700	//5330正常弹丸	//5480荧光弹丸//5130新45a摩擦轮  //4700
#else
#define INITIAL_FRIC_SPEED 4700	//5330正常弹丸	//5650荧光弹丸//5130新45a摩擦轮  5680 调试用小的       //工培温度5700//三楼温度5450
#endif
#define TARGET_BULLET_SPEED 15.10
//#define STIR_MECHANICAL_ANGLE_TO_DEGREE(MECHANICAL_ANGLE) (MECHANICAL_ANGLE) / 8191.0f * 360 / 19			//机械角转3508角度


/**
 * @brief 底盘控制相关结构体，存放闭环控制器，目标赋值等等
 */
typedef struct
{
	/*底盘跟随控制相关*/
	struct
	{
		PIDStruct follow_speed_need_pid;		//以差角为pid输入算所需的跟随速度
		int8_t revolve_return_flag;           //用来判断自旋恢复方式的flag
	}ChassisFollowControl;
	/*云台坐标系下的目标速度*/
	struct
	{
		float speed_x_mps;
		float speed_y_mps;
		float max_revolve_speed_rps;
		//加底盘整体速度的反馈控制
		PIDStruct speed_x_compensate_pid;
		PIDStruct speed_y_compensate_pid;		
	}GimbalCoordinateInput;
	/*底盘坐标系下的目标速度*/
	struct
	{
		float speed_x_mps;	// m/s
		float speed_y_mps;	// m/s
		float speed_w_rps;	// rps
		float compensate_speed_w_dps;//dps
	}ChassisCoordinateInput;
	/*底盘实际需要的目标速度，从目标速度赋值过来后，经过功率限制，反馈补偿可能会改变*/
	struct
	{
		float speed_x_mps;	// m/s
		float speed_y_mps;	// m/s
		float speed_w_rps;	// rps
		float power_limit_scale;	//底盘功率限制缩减设定底盘速度系数
		float compensate_power;		//电容补偿功率，可能有正有负，为负时即为惩罚功率，当电容电压过低时可能会考虑
	}ChassisRealNeedInput;
	/*轮电机的目标转速及控制*/
	struct
	{
		float target_speed_mps[4];	// m/s
		int16_t target_motor_output[4];
		PIDStruct speed_control_pid[4];
	}WheelMotorControl;
	/*底盘观测真实数据*/
	struct
	{
		float gimbal_to_chassis_delta_angle_d;	//云台坐标系与底盘坐标系的差角，degree
		float chassis_follow_angle_d;			//底盘跟随云台角
		float wheel_real_speed_mps[4];		//轮速，m/s
		//根据当前轮速解算出的真实底盘速度，m/s
		float speed_x_mps;	// m/s
		float speed_y_mps;	// m/s
		float speed_w_rps;	// rps
		/*欧拉角解算出底盘角度*/
		float yaw_d;
		float pitch_d;
		float roll_d;
		/*IMU底盘平地近似出的角速度*/
		float imu_yaw_dps;
	}ChassisEstimate;
	struct
	{
		int16_t total_output_power;
		int16_t state;
		float max_compensate_power;
	}SuperCapacity;
}ChassisControl;

 
/**
 * @brief 云台控制相关结构体
 */
typedef struct
{
	/*云台目标输入*/
	struct
	{
		float pitch_angle_d;	//大pitch目标角
		float small_pitch_angle_d;
		float yaw_angle_d;		//期望的yaw目标角
		float pitch_angular_velocity_dps; //pitch期望角速度
		float yaw_angular_velocity_dps; //yaw期望角速度
		float yaw_recoil_compensation_d;
	}GimbalTargetInput;
	/*云台控制相关*/
	struct
	{
		/*控制器*/
		PIDStruct pitch_calibration_pid;
		
		ADRC pitch_angle_adrc;
		PIDStruct pitch_speed_pid;
		
		LTD pitch_LTD;//测试用,7.5分
		LTD yaw_LTD;
		
		LTDPID pitch_LTD_pid;
		LTDPID yaw_LTD_pid;
		
//		PIDStruct small_pitch_angle_pid;
//		PIDStruct small_pitch_speed_pid;
		
//		ADRC yaw_angle_adrc;		
//		PIDStruct yaw_speed_pid;
//		PIDStruct yaw_angle_pid;

		/*内环输入*/
//		float pitch_angular_velocity_dps; //pitch内环目标角速度
//		float yaw_angular_velocity_dps; //yaw内环目标角速度		
		/*PID输出*/
		float yaw_PID_output;
		/*前馈部分*/
		float yaw_qian_kui;
		/*电机实际输出值*/
		int16_t yaw_target_output;
		int16_t pitch_target_output;	
		//		float small_pitch_velocity_output;因为使用了内部闭环
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
		float robot_slope_angle;//机器人整体倾斜角度，没有用旋转矩阵计算的
		float yaw_angle_d;
		float yaw_angular_velocity_dps;	
		
		float roll_angle_d;
		float roll_angular_velocity_dps;
		//6.22 前馈准备
//		float yaw_speed_error;
//		float yaw_w;
	}GimbalEstimate;
}GimbalControl;

 
/**
 * @brief 发射控制相关结构体
 */
typedef struct
{
	struct
	{
		float fric_speed_rpm[6];           // hxg
		float stir_speed_rps;
		float stir_angle_d;		//stir预期角度
		float stir_target_pos;
		float stir_target_pos_rad;
		float stir_target_vol;
		
		float stir_all_target_pos_d;//发现可以直接输入多圈
		float stir_all_target_pos_rad;
		int shoot_flag;
		int shoot_cnt;
	}ShootTargetInput;
	
	struct
	{
		PIDStruct fric_speed_pid[6];
		int16_t fric_target_output[6];
		
		float stir_preset_angle;
		float stir_angular_velocity_dps; //stir内环目标角速度
		
	}ShootMotorControl;
		
	struct
	{
		uint8_t stir_block_flag;
		uint8_t stir_reset_flag;
		uint8_t stir_enableflag_detect;//pitch实际使能状态
		uint8_t stir_enableflag_desire; //pitch期望使能状态
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

void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w,float roll_angle,float roll_angle_w);

/* ====================================================================================================================
 * 世界系云台控制结构体 WorldGimbal
 * 原理：维护虚拟目标指向 f_des_B（在机体坐标系B中），用户yaw指令绕大地竖直方向 g_B 旋转 f_des_B，
 *       pitch指令绕虚拟水平右轴旋转 f_des_B，再用阻尼最小二乘IK反解真实yaw/pitch电机角。
 * 坐标系：B系 x=前 y=右 z=下（右手系）
 * 旋转顺序：Z-Y-X (Yaw→Pitch→Roll)，与 rotation_Martix.h 一致
 * ==================================================================================================================== */
#define WORLDGIMBAL_IK_MAX_ITERS      10      /* IK最大迭代次数 */
#define WORLDGIMBAL_IK_LAMBDA         0.01f   /* 阻尼因子 */
#define WORLDGIMBAL_IK_MAX_STEP_RAD   0.05f   /* 每步最大步长 (rad) */
#define WORLDGIMBAL_IK_CONVERGE_RAD   0.0005f /* 收敛阈值 (rad, ≈0.03°) */

typedef struct
{
    /* ---- 世界系目标输入（DecisionTask 中更新） ---- */
    struct
    {
        float f_des_B[3];           /* 虚拟目标指向在机体坐标系中的单位向量 */
        float last_right_B[3];      /* 上一帧有效右轴（pitch奇异点保护） */
        uint8_t last_right_valid;   /* last_right_B 是否有效 */
        uint8_t init_done;          /* 初始化完成标志 */
    } WorldGimbalTargetInput;

    /* ---- 世界系观测量（ControlTask 中更新） ---- */
    struct
    {
        float g_B[3];               /* 大地竖直向下方向在机体坐标系中的表达（归一化） */
        float chassis_roll_deg;     /* 底盘roll (deg), 来自485下板IMU */
        float chassis_pitch_deg;    /* 底盘pitch (deg), 来自485下板IMU */
        float chassis_yaw_deg;      /* 底盘yaw (deg), 来自485下板IMU（仅用于调试，不参与g_B计算） */
        float f_real_B[3];          /* 当前真实指向（由电机编码器正运动学FK计算） */
        float b_B[3];               /* 当前pitch轴在机体坐标系中的方向 */
        float angle_error_deg;      /* 虚拟目标指向与真实指向之间的角度误差 (deg) */
        float world_yaw_deg;        /* 世界系azimuth: f_des_B在水平面投影的方向角 (deg, 相对底盘正向投影) */
        float world_pitch_deg;      /* 世界系elevation: f_des_B相对水平面的仰角 (deg, 正=向上) */
    } WorldGimbalEstimate;

    /* ---- IK反解输出（ControlTask 中更新，写入 GimbalTargetInput） ---- */
    struct
    {
        float q_yaw_cmd_deg;        /* IK解算的yaw电机目标角 (deg, 相对底盘正向) */
        float q_pitch_cmd_deg;      /* IK解算的pitch电机目标角 (deg, 相对水平) */
        float q_yaw_cmd_rad;        /* 同上，弧度 */
        float q_pitch_cmd_rad;      /* 同上，弧度 */
        uint8_t converged;          /* IK本周期是否收敛 */
        uint8_t iters_used;         /* 实际使用的迭代次数 */
    } WorldGimbalControl;

    /* ---- 使能标志 ---- */
    uint8_t enable;                 /* 世界系控制使能 */
} WorldGimbal;

/* API */
void WorldGimbalInit(WorldGimbal* wg);
void WorldGimbalAlignToCurrent(WorldGimbal* wg);
void WorldGimbalInputUpdate(WorldGimbal* wg, float dyaw_deg, float dpitch_deg);
void WorldGimbalEstimateUpdate(WorldGimbal* wg);
void WorldGimbalIKSolve(WorldGimbal* wg);
void WorldGimbalApplyToTargets(WorldGimbal* wg);
void WorldGimbalSetWorldAngles(WorldGimbal* wg, float world_yaw_deg, float world_pitch_deg);

extern WorldGimbal worldGimbal;
extern const WorldGimbal* _worldGimbal;

#endif

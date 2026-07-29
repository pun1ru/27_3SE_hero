#ifndef _PERIPHERAL_RECEIVE_TASK_H_
#define _PERIPHERAL_RECEIVE_TASK_H_

#include <stdint.h>

#include "cmsis_compiler.h"

/*遥操作接收数据来源事件组BIT宏定义*/
#define EVENT_GROUP_BIT_ERROR 			(1UL << 0UL)	//传输错误,一段时间内都未接收到正确信号
#define EVENT_GROUP_BIT_DT7 			(1UL << 1UL)	//DT7
#define EVENT_GROUP_BIT_VT3				(1UL<<2UL) //遥控器或者别的什么遥控器比如VT3
/**
 * @brief 当前遥操作信号来源编号
 */
typedef enum
{ERROR_RECEIVE=0, DT7,VT13}RemoteSourceEnum;

/**
 * @brief 由于遥操作数据可能来源于不同遥控器或自定义控制器，为了robotcontrol中拿到的信息能统一，
 *		  将不同遥控器的键位信息归化到统一的键位信息结构体中，便于读取
 */
typedef struct
{
	uint8_t remote_source;
	__PACKED_STRUCT
	{
		unsigned switch_L1   : 2;
		unsigned switch_R1   : 2;
		unsigned reserved 	 : 4;
	}Switch;
	
	__PACKED_STRUCT
	{
		unsigned level_key_W : 1;
		unsigned level_key_A : 1;
		unsigned level_key_S : 1;
		unsigned level_key_D : 1;
		unsigned level_key_SHIFT : 1;
		unsigned level_key_CTRL  : 1;
		unsigned level_key_Q : 1;
		unsigned level_key_E : 1;
		
		unsigned level_key_R : 1;
		unsigned level_key_F : 1;
		unsigned level_key_G : 1;
		unsigned level_key_Z : 1;	
		unsigned level_key_X : 1;
		unsigned level_key_C : 1;
		unsigned level_key_V : 1;
		unsigned level_key_B : 1;
	}PCKeyBoard;
	struct
	{
		float ch0, ch1, ch2, ch3, ch4;
	}RelativeCH;
	struct
	{
		uint8_t mouse_left;
		uint8_t mouse_right;
		int16_t mouse_speed_x;
		int16_t mouse_speed_y;
		int16_t mouse_speed_z;
	}PCMouse;
}NormRemoteCmd;

extern const NormRemoteCmd* _normRemoteCmd;
extern volatile float gimbal_yaw_rx_d;
extern volatile float gimbal_yaw_dps_rx;
extern volatile float gimbal_pitch_rx_d;
extern volatile float gimbal_pitch_dps_rx;
extern volatile float gimbal_yaw_target_rx_d;
extern volatile uint8_t gimbal_yaw_rx_valid;
extern volatile int16_t gimbal_fric_rpm_rx_arr[6];

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief DJI电机接收结构体
 */
typedef struct
{
	uint16_t 	frame_counter;
	int16_t 	torque_current_real;
	uint16_t 	mechanical_angle;
	int16_t		mechanical_speed_rpm;
	uint8_t 	motor_temperature_d;	
}DJIGMotorRec;

/// @brief 达妙电机接收结构体
typedef struct
{
	uint16_t 	frame_counter;
	uint32_t  timestamp;
	int state;//状态 1OK 12过热
	int p_int;//角度
	int v_int;//速度
	int t_int;
	int kp_int;
	int kd_int;
	
	int id;
	float pos_d;
	float vel_radps;
	float toq;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;

}DMJ4310MotorRec;

extern const DMJ4310MotorRec* _stirMotorRec;
extern const DMJ4310MotorRec* _jointMotorEec;
extern const DMJ4310MotorRec* _caterpillarMotorRec;
extern const DJIGMotorRec* _fricMotorRec;

typedef struct
{
	uint16_t frame_counter;
	float compensated_power;
	float cap_volt;
	float power_limit;
	float real_power;
}SuperCapacity;

extern const DJIGMotorRec* _chassisMotorRec;
extern const SuperCapacity* _superCapacity;


/// @brief 瓴控电机接收结构体
typedef struct 
{
	int8_t temperature;
	int16_t iq;
	float speed_rps;
	uint16_t encoder;
	uint8_t frame_counter;
}LKMotorRec;

//裁判系统弹速统计结构体
typedef struct 
{
	uint8_t record_shoot_count;//已发弹记录数据个数
	float median;
	float hit_rate_median;
	float relative_accuracy_median;
	float absolute_accuracy_median;
	
	float mode;
	float hit_rate_mode;
	float relative_accuracy_mode;
	float absolute_accuracy_mode;
	
	float average;
	float hit_rate_average;
	float relative_accuracy_average;
	float absolute_accuracy_average;
	float bullet_speed[30];
	float predict_speed;
}DataFromJudge;
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#define HEAT_MAX 5000-1
#define HEAT_MIN 500
#define HEAT_MID 3500
#define IMU_TARGET_TEMPERATURE 40
 
#define ONBOARD_EKF_SOLVE //开启板载imu解算

/**
 * @brief 姿态记录结构体
 */
typedef struct
{
	/*欧拉角姿态，单位为degree*/
	float pitch_d;
	float yaw_d;
	float roll_d;
	
	/*绕xyz三轴的角速度，单位为rad/s*/
	float pitch_radps;
	float roll_radps;
	float yaw_radps;

	/*IMU坐标系下的线性加速度，单位为m/s²*/
	float accel_x;
	float accel_y;
	float accel_z;
}Pose;

extern const Pose* _gimbalPose;


void PeripheralRecEnable(void);
void RemoteRecRestart(void);

/**
 * @brief   初始化遥控接收任务所需的共享资源
 * @param   void
 * @retval  void
 */
void RemoteRecInitialize(void);
void RemoteRecTask(void* argument);
void IMUTask(void* argument);
void UpperPCCommTask(void* argument);

#endif

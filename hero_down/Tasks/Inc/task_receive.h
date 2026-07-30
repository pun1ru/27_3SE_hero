#ifndef TASK_RECEIVE_H_
#define TASK_RECEIVE_H_

#include <stdint.h>

#include "cmsis_compiler.h"

#include "DM_driver.h"
#include "VT13_rc_ctrl.h"


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


extern const DMMotorRec* _stirMotorRec;
extern const DMMotorRec* _jointMotorEec;
extern const DMMotorRec* _caterpillarMotorRec;
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

/**
 * @brief   处理算法上位机通过 USB CDC 发来的数据帧
 * @param   rec_buf 接收数据缓冲区
 * @param   size 接收数据长度，单位为字节
 * @retval  void
 */
void UpperCommRecHandler(const uint8_t* rec_buf, uint32_t size);

void UpperPCCommTask(void* argument);

#endif

#ifndef _PERIPHERAL_RECEIVE_TASK_H_
#define _PERIPHERAL_RECEIVE_TASK_H_

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
	__packed struct
	{
		unsigned switch_L1   : 2;
		unsigned switch_R1   : 2;
		unsigned reserved 	 : 4;
	}Switch;
	
	__packed struct{
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

typedef struct
{
	uint16_t frame_counter;
	float compensated_power;
	float cap_volt;
	float power_limit;
	float real_power;
}SuperCapacity;


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
}Pose;


void PeripheralRecEnable(void);
void RemoteRecRestart(void);

/* 485双板通信：下板传输的裁判系统血量数据 */
extern volatile uint16_t g_b2b_current_hp;
extern volatile uint8_t g_b2b_hp_zero_flag;

#endif

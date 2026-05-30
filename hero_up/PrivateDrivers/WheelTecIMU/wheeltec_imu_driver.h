#ifndef _WHEEL_TEC_IMU_
#define _WHEEL_TEC_IMU_

#include "stdint.h"

#define MSG_IMU_ID 0x40
#define MSG_AHRS_ID 0x41
#define MSG_IMU_LENGTH 64
#define MSG_AHRS_LENGTH 56
#define MSG_HEADER 0xFC
#define MSG_TAIL 0xFD

typedef struct
{
	struct
	{
		float gyro_x;
		float gyro_y;
		float gyro_z;
		
		float accel_x;
		float accel_y;
		float accel_z;
		
		float mag_x;
		float mag_y;
		float mag_z;
		
		float temperature;//imu内部平均温度
		
		float pressure;//	气压值 单位Pa
		float pressure_temperature;//气压计温度
		
		int64_t time_stamp;
	}MsgIMU;
	
	struct
	{
		float roll_speed_radps;
		float pitch_speed_radps;
		float yaw_speed_radps;
		
		float roll;
		float pitch;
		float yaw;
		
		float q0;
		float q1;
		float q2;
		float q3;
		
		int32_t time_stamp;
	}MsgAHRS;
	
}WheelTecIMURec;


float Uint8ToFloatFunc(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4);
void WheelTecIMUDecode(uint8_t* raw_data, WheelTecIMURec* rec_imu);
#endif

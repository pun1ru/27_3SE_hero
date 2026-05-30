#include "math.h"

#include "wheeltec_imu_driver.h"

float Uint8ToFloatFunc(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4)
{
	long long transition_32;
	float tmp = 0;
	int sign = 0;
	int exponent = 0;
	float mantissa = 0;
	transition_32 = 0;
	transition_32 |= data_4 << 24; //得到数据的底 8 位
	transition_32 |= data_3 << 16;
	transition_32 |= data_2 << 8;
	transition_32 |= data_1; //得到数据的高 8 位
	sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
	//先右移操作，再按位与计算，出来结果是 30 到 23 位对应的 e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//将 22~0 转化为 10 进制，得到对应的 x 系数
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp = sign * mantissa * pow(2, exponent);
	return tmp;
}


void WheelTecIMUDecode(uint8_t* raw_data, WheelTecIMURec* rec_imu)
{
//	if(raw_data[0] == MSG_HEADER)//帧头校验
//	{	
//		if(raw_data[1] == MSG_IMU_ID && raw_data[2] == MSG_IMU_LENGTH && raw_data[MSG_IMU_LENGTH - 1] == MSG_TAIL)
//		{
			rec_imu->MsgIMU.gyro_x = Uint8ToFloatFunc(raw_data[7], raw_data[8], raw_data[9], raw_data[10]);
			rec_imu->MsgIMU.gyro_y = Uint8ToFloatFunc(raw_data[11], raw_data[12], raw_data[13], raw_data[14]);
			rec_imu->MsgIMU.gyro_z = Uint8ToFloatFunc(raw_data[15], raw_data[16], raw_data[17], raw_data[18]);
			rec_imu->MsgIMU.accel_x = Uint8ToFloatFunc(raw_data[19], raw_data[20], raw_data[21], raw_data[22]);
			rec_imu->MsgIMU.accel_y = Uint8ToFloatFunc(raw_data[23], raw_data[24], raw_data[25], raw_data[26]);
			rec_imu->MsgIMU.accel_z = Uint8ToFloatFunc(raw_data[27], raw_data[28], raw_data[29], raw_data[30]);
			rec_imu->MsgIMU.mag_x = Uint8ToFloatFunc(raw_data[31], raw_data[32], raw_data[33], raw_data[34]);
			rec_imu->MsgIMU.mag_y = Uint8ToFloatFunc(raw_data[35], raw_data[36], raw_data[37], raw_data[38]);
			rec_imu->MsgIMU.mag_z = Uint8ToFloatFunc(raw_data[39], raw_data[40], raw_data[41], raw_data[42]);
			rec_imu->MsgIMU.temperature = Uint8ToFloatFunc(raw_data[43], raw_data[44], raw_data[45], raw_data[46]);
			rec_imu->MsgIMU.pressure = Uint8ToFloatFunc(raw_data[47], raw_data[48], raw_data[49], raw_data[50]);
			rec_imu->MsgIMU.pressure_temperature = Uint8ToFloatFunc(raw_data[51], raw_data[52], raw_data[53], raw_data[54]);
			rec_imu->MsgIMU.time_stamp = Uint8ToFloatFunc(raw_data[55], raw_data[56], raw_data[57], raw_data[58]);
//		}
//		
//		else if(raw_data[1] == MSG_AHRS_ID && raw_data[2] == MSG_IMU_LENGTH && raw_data[MSG_AHRS_LENGTH - 1] == MSG_TAIL)
//		{
			rec_imu->MsgAHRS.roll_speed_radps = Uint8ToFloatFunc(raw_data[7], raw_data[8], raw_data[9], raw_data[10]);
			rec_imu->MsgAHRS.pitch_speed_radps = Uint8ToFloatFunc(raw_data[11], raw_data[12], raw_data[13], raw_data[14]);
			rec_imu->MsgAHRS.yaw_speed_radps = Uint8ToFloatFunc(raw_data[15], raw_data[16], raw_data[17], raw_data[18]);
			rec_imu->MsgAHRS.roll = Uint8ToFloatFunc(raw_data[19], raw_data[20], raw_data[21], raw_data[22]);
			rec_imu->MsgAHRS.pitch = Uint8ToFloatFunc(raw_data[23], raw_data[24], raw_data[25], raw_data[26]);
			rec_imu->MsgAHRS.yaw = Uint8ToFloatFunc(raw_data[27], raw_data[28], raw_data[29], raw_data[30]);
			rec_imu->MsgAHRS.q0 = Uint8ToFloatFunc(raw_data[31], raw_data[32], raw_data[33], raw_data[34]);
			rec_imu->MsgAHRS.q1 = Uint8ToFloatFunc(raw_data[35], raw_data[36], raw_data[37], raw_data[38]);
			rec_imu->MsgAHRS.q2 = Uint8ToFloatFunc(raw_data[39], raw_data[40], raw_data[41], raw_data[42]);
			rec_imu->MsgAHRS.q3 = Uint8ToFloatFunc(raw_data[43], raw_data[44], raw_data[45], raw_data[46]);
			rec_imu->MsgAHRS.time_stamp = Uint8ToFloatFunc(raw_data[47], raw_data[48], raw_data[49], raw_data[50]);
//		}
//	}
}

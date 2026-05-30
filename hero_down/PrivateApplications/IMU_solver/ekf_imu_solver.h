#ifndef _EKF_IMU_SOLVE_H_
#define _EKF_IMU_SOLVE_H_

#include "stdint.h"

#define USE_ONBOARD_IMU_DATA
//#define USE_EXTERNAL_IMU_DATA

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll_d;
    float Pitch_d;
    float Yaw_d;
}IMUUseEKFSolver;

/**
 * @brief mpu接收到的bmi088和ist8310原始数据
 */
typedef struct
{
	/*传感器原始数据*/
	int16_t accel_raw[3];
	int16_t gyro_raw[3];
	float temperature;
	
	/*传感器数据换算到具有物理单位的数据*/
	float accel[3];
	float gyro[3];
	
	/*imu实时温度*/
	int16_t temperature_raw;
	
	/*imu传感器偏置值，计算方法为上电时取若干周期传感器原始数据取平均值，新的计算方法为精致的时候测一下*/
	float gyro_offset[3];	
	float accel_offset[3];
	float g_norm;
	float accel_scale;//=9.81f * real_gravity
}IMURecData;

static void GetIMUOffset(IMURecData* imu_data_rec);
static void UpdateIMUData(IMURecData* imu_data_rec);
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

/*API function*/
void IMUSolverUseEKFInitialize(IMUUseEKFSolver* imu, IMURecData* imu_data_rec, float dt_set);
void IMUSolverUseEKFUserFunc(IMUUseEKFSolver* imu,IMURecData* imu_data_rec);
#endif 

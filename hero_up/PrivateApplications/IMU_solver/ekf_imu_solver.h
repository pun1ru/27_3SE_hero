#ifndef _EKF_IMU_SOLVE_H_
#define _EKF_IMU_SOLVE_H_

#include "stdint.h"

#define USE_ONBOARD_IMU_DATA
//#define USE_EXTERNAL_IMU_DATA

/* ===== IMU 校准参数配置 ===== */
#define IMU_CALIB_TARGET_TEMP   39.0f   /* 目标温度（摄氏度），达到后才开始采样 */
#define IMU_CALIB_SAMPLE_COUNT  5000    /* 有效采样总数 */
#define IMU_CALIB_DECIMATION    5       /* 每N个原始数据取1个有效样本 */

/* ===== IMU 校准状态机 ===== */
typedef enum
{
    IMU_CALIB_WARMUP = 0,   /* 预热阶段：等待IMU温度达到设定值 */
    IMU_CALIB_SAMPLING,     /* 采样阶段：采集5000个有效样本 */
    IMU_CALIB_DONE          /* 校准完成：偏置值已写入 imuRecData */
} IMU_CalibState;

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
	
	/* ===== IMU偏置校准实时状态（全局可观测，方便Debug） ===== */
	IMU_CalibState calib_state;         /* 当前校准阶段：WARMUP / SAMPLING / DONE */
	uint16_t calib_sample_count;        /* 已采集的有效样本数（0 ~ 5000） */
	uint16_t calib_read_count;          /* 采样阶段BMI088总读取次数（含被decimation跳过的） */
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

/**
 * @brief  IMU偏置校准状态机（每个IMU任务周期调用一次，非阻塞）
 * @param  imu_data_rec: IMU数据结构指针，校准完成后偏置值写入其中
 * @retval 当前校准状态（IMU_CALIB_DONE表示校准完成）
 * @note   调用频率 = IMU_TASK_PERIOD_SET (500Hz)
 *         预热阶段：等待温度 >= 40°C
 *         采样阶段：每5个数据取1个，累计5000个有效样本
 *         完成后自动计算 gyro_offset[3]、g_norm、accel_scale
 */
IMU_CalibState IMU_CalibProcess(IMURecData* imu_data_rec);
#endif 

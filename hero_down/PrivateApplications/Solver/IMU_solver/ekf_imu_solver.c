/**
 ******************************************************************************
 * @file    ekf_imu_solver.c
 * @author  Chen qiyuan
 * @version V1.0.0
 * @date    2023/11/28
 * @brief
 ******************************************************************************
 * @note open source reference: https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example
 *
 ******************************************************************************
 */
#include "Notch_filter.h"
#include "ekf_imu_solver.h"
#include "ekf_quaternion.h"
#include "bmi088_driver.h"

#define X 0
#define Y 1
#define Z 2

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
static float dt = 0;


 // 初始化带阻滤波器的全局变量
Biquad Nfilter[3];
Biquad Nfiltera[3];
BiquadLPF gyrolpf[3];
BiquadLPF gyrolpf1[3];
BiquadLPF gyrolpf2[3];
BiquadLPF acclpf[3];
FirstOrderHighPassFilterState_t filter_gao[3];
FirstOrderHighPassFilterState_t filter_gao_a[3];
//低通滤波器
//BiquadLPF lpf_40hz[3];
//BiquadLPF lpf_40hz_a[3];
float temp_see[3]={0};
void IMUSolverUseEKFInitialize(IMUUseEKFSolver* imu, IMURecData* imu_data_rec, float dt_set)
{		//float fc = 40.0f;    // 截止频率40Hz
		//const float Qb = 0.707f;    // 巴特沃斯参数
    float fs = 500.0;           // 采样率 500Hz
    float notchFreq = 79;     // 带阻中心频率81 91
    float Q = 8;               // 品质因数
    // 初始化带阻滤波器
	//初始化低通滤波器 陷波滤波器


	//准备试一下角速度带阻滤波,过滤79hz的摩擦轮噪声
	//加速度计用低通滤波,截止频率40hz,巴特沃斯响应
	for(int i=0;i<3;i++){
//		initLowPassFilter(&lpf_40hz[i], fs, fc, Q);
//		initLowPassFilter(&lpf_40hz_a[i], fs, fc, Q);

		initBandStopFilter(&Nfilter[i], fs, notchFreq, Q);
		initBandStopFilter(&Nfiltera[i], fs, notchFreq, Q);
		initLowPassFilter(&gyrolpf[i], fs, 40.0f, 0.707f);
		//级联巴特沃斯,不行
		initLowPassFilter(&gyrolpf[i], fs, 40.0f, 0.541f);
		initLowPassFilter(&gyrolpf[i], fs, 40.0f, 1.306f);
		
		initLowPassFilter(&acclpf[i], fs, 40.0f, 0.707f);
		HighPassFilter_Init(&filter_gao[i]);
		HighPassFilter_Init(&filter_gao_a[i]);
	}
	//此处可以EKF调参
//  IMU_QuaternionEKF_Init(1, 0.0005, 10000000, 0.99,0);//最后一个是lpf QEKF_INS.accLPFcoef = 6.6923f * (IMU_TASK_PERIOD_SET / 1000.0f);
	IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1,0);
	dt = dt_set;
  imu->AccelLPF = 0.0085;
	GetIMUOffset(imu_data_rec);
}
float ditong[3]={0};
float ditonga[3]={0};
void IMUSolverUseEKFUserFunc(IMUUseEKFSolver* imu, IMURecData* imu_data_rec)
{

    const float gravity[3] = {0, 0, 9.81f};	
	UpdateIMUData(imu_data_rec);

//task02,imu滤波
	/*把滤波移动到这里*/
	/*额外增加的角速度滤波*/
//		QEKF_INS.gyroLPFcoef=0.87;
		
		for(int i=0;i<3;i++){
//			imu->Gyro[i]=processLPF(&lpf_40hz[i], imu_data_rec->gyro[i]);
//			imu->Accel[i]=processLPF(&lpf_40hz_a[i], imu_data_rec->accel[i]);	
			imu->Gyro[i]=imu_data_rec->gyro[i];
			imu->Accel[i]=imu_data_rec->accel[i];
			// imu->Accel[i] = processSample(&Nfiltera[i],imu_data_rec->accel[i]);
//			imu->Gyro[i] = processLPF(&gyrolpf[i],imu_data_rec->gyro[i]);
//			imu->Gyro[i] = processLPF(&gyrolpf1[i],imu_data_rec->gyro[i]);
//			imu->Gyro[i] = processLPF(&gyrolpf2[i],imu->Gyro[i]);
//			imu->Accel[i] = processLPF(&acclpf[i],imu_data_rec->accel[i]);
//			imu->Gyro[i]=imu_data_rec->gyro[i];
//			imu->Gyro[i] = HighPassFilter_Process(&filter_gao[i],imu->Gyro[i]);
//			imu->Accel[i] = HighPassFilter_Process(&filter_gao_a[i],imu->Accel[i]);
			
			
//			   imu->Gyro[i] = processSample(&Nfilter[i],imu_data_rec->gyro[i]);
//				 imu->Accel[i]=imu_data_rec->accel[i];
//				 ditong[i]=ditong[i]*0.615+imu->Gyro[i]*0.385;
//				 imu->Gyro[i]=ditong[i];
//				 ditonga[i]=ditonga[i]*0.7+imu->Accel[i]*0.3;
//				 imu->Accel[i]=ditonga[i];
			
			
//				 temp_see[i]=imu->Gyro[i];
		}
	// 核心函数,EKF更新四元数
	IMU_QuaternionEKF_Update(imu->Gyro[X], imu->Gyro[Y], imu->Gyro[Z], imu->Accel[X], imu->Accel[Y], imu->Accel[Z], dt);

	memcpy(imu->q, QEKF_INS.q, sizeof(QEKF_INS.q)); 

	// 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
	BodyFrameToEarthFrame(xb, imu->xn, imu->q);
	BodyFrameToEarthFrame(yb, imu->yn, imu->q);
	BodyFrameToEarthFrame(zb, imu->zn, imu->q);

	// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
	float gravity_b[3];
	EarthFrameToBodyFrame(gravity, gravity_b, imu->q);
	for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
	{
		imu->MotionAccel_b[i] = (imu->Accel[i] - gravity_b[i]) * dt / (imu->AccelLPF + dt) + imu->MotionAccel_b[i] * imu->AccelLPF / (imu->AccelLPF + dt);
	}
	BodyFrameToEarthFrame(imu->MotionAccel_b, imu->MotionAccel_n, imu->q); // 转换回导航系n

	// 获取最终数据
	imu->Yaw_d = QEKF_INS.Yaw;
	imu->Pitch_d = QEKF_INS.Pitch;
	imu->Roll_d = QEKF_INS.Roll;
}

/**
 * @brief  更新imu偏置值
 */

#define GYRO_X_OFFSET_DEFAULT 0.003541457
#define GYRO_Y_OFFSET_DEFAULT -0.000740358082
#define GYRO_Z_OFFSET_DEFAULT 0.000596015656
#define GRAVITY_OFFSET_DEFAULT 9.7906065

static void GetIMUOffset(IMURecData* imu_data_rec)
{
	BMI088_init();
	
	int16_t gyro_rec_temp[3];
	int16_t accel_rec_temp[3];
	//2026.5.22 实测值 (温度39°C, 5000样本)
	imu_data_rec->gyro_offset[X]=0.00122206332f;
	imu_data_rec->gyro_offset[Y]=0.000610821124f;
	imu_data_rec->gyro_offset[Z]=0.00302873505f;
//	imu_data_rec->gyro_offset[X]=0.0026825;
//	imu_data_rec->gyro_offset[Y]=0.00071103;
//	imu_data_rec->gyro_offset[Z]=-0.0033635;
	imu_data_rec->accel_offset[X]=0;
	imu_data_rec->accel_offset[Y]=0;
	imu_data_rec->accel_offset[Z]=0;
//	for (uint16_t i = 0; i < 2000; i++)//看看要不要测一下或者实时算呢,
//	{
//		#ifdef USE_ONBOARD_IMU_DATA
//			BMI088_read(gyro_rec_temp, accel_rec_temp, &imu_data_rec->temperature_raw);
//		#endif
//		
//		HAL_Delay(1);
//		
//		imu_data_rec->accel[X] = accel_rec_temp[X] * BMI088_ACCEL_SEN;
//		imu_data_rec->accel[Y] = accel_rec_temp[Y] * BMI088_ACCEL_SEN;
//		imu_data_rec->accel[Z] = accel_rec_temp[Z] * BMI088_ACCEL_SEN;
//		
//		imu_data_rec->g_norm += sqrt(imu_data_rec->accel[X] * imu_data_rec->accel[X] + \
//									 imu_data_rec->accel[Y] * imu_data_rec->accel[Y] + \
//									 imu_data_rec->accel[Z] * imu_data_rec->accel[Z]);

//		imu_data_rec->gyro_offset[X] += gyro_rec_temp[X] * BMI088_GYRO_SEN;
//		imu_data_rec->gyro_offset[Y] += gyro_rec_temp[Y] * BMI088_GYRO_SEN;
//		imu_data_rec->gyro_offset[Z] += gyro_rec_temp[Z] * BMI088_GYRO_SEN;
//		
//	
//	}
//	
//	imu_data_rec->g_norm /= 2000.0f;
//	imu_data_rec->gyro_offset[X] /= 2000.0f;
//	imu_data_rec->gyro_offset[Y] /= 2000.0f;
//	imu_data_rec->gyro_offset[Z] /= 2000.0f;
//	imu_data_rec->accel_scale = 9.81f / imu_data_rec->g_norm;
//	
//	/*default*/
//	if(fabs(imu_data_rec->gyro_offset[X] - GYRO_X_OFFSET_DEFAULT) > 0.1) imu_data_rec->gyro_offset[X] = GYRO_X_OFFSET_DEFAULT;
//	if(fabs(imu_data_rec->gyro_offset[Y] - GYRO_Y_OFFSET_DEFAULT) > 0.1) imu_data_rec->gyro_offset[Y] = GYRO_Y_OFFSET_DEFAULT;
//	if(fabs(imu_data_rec->gyro_offset[Z] - GYRO_Z_OFFSET_DEFAULT) > 0.1) imu_data_rec->gyro_offset[Z] = GYRO_Z_OFFSET_DEFAULT;
//	if(fabs(imu_data_rec->g_norm - 9.81f) > 0.2) imu_data_rec->g_norm = GRAVITY_OFFSET_DEFAULT;
//	
//	imu_data_rec->accel_scale = 9.81f / imu_data_rec->g_norm;	
//	imu_data_rec->temperature = BMI088_TEMP_OFFSET + imu_data_rec->temperature_raw * BMI088_TEMP_FACTOR;
}

/**
 * @brief 更新imu当前数据
 * @param imu接收数据
 */
static void UpdateIMUData(IMURecData* imu_data_rec)
{
	BMI088_read(&imu_data_rec->gyro_raw[0], &imu_data_rec->accel_raw[0], &imu_data_rec->temperature_raw);
	
	imu_data_rec->gyro[X] = imu_data_rec->gyro_raw[X] * BMI088_GYRO_SEN - imu_data_rec->gyro_offset[X];
	imu_data_rec->gyro[Y] = imu_data_rec->gyro_raw[Y] * BMI088_GYRO_SEN - imu_data_rec->gyro_offset[Y];//YAW轴仍然有零漂，也不知道怎么办
	imu_data_rec->gyro[Z] = imu_data_rec->gyro_raw[Z] * BMI088_GYRO_SEN - imu_data_rec->gyro_offset[Z];
	
//	imu_data_rec->gyro[X] = imu_data_rec->gyro_raw[X] * BMI088_GYRO_SEN;
//	imu_data_rec->gyro[Y] = imu_data_rec->gyro_raw[Y] * BMI088_GYRO_SEN;//YAW轴仍然有零漂，也不知道怎么办
//	imu_data_rec->gyro[Z] = imu_data_rec->gyro_raw[Z] * BMI088_GYRO_SEN;
	
	imu_data_rec->accel[X] = imu_data_rec->accel_raw[X] * BMI088_ACCEL_SEN - imu_data_rec->accel_offset[X];
	imu_data_rec->accel[Y] = imu_data_rec->accel_raw[Y] * BMI088_ACCEL_SEN - imu_data_rec->accel_offset[Y];
	imu_data_rec->accel[Z] = imu_data_rec->accel_raw[Z] * BMI088_ACCEL_SEN - imu_data_rec->accel_offset[Z];
	
	imu_data_rec->temperature = BMI088_TEMP_OFFSET + imu_data_rec->temperature_raw * BMI088_TEMP_FACTOR;
	
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

/**
 ******************************************************************************
 * @file    new_ekf.c
 * @brief   Coupled gimbal/chassis quaternion EKF implementation
 ******************************************************************************
 */
#include "new_ekf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define NEW_EKF_GIMBAL_Q_OFFSET         NEW_EKF_STATE_GIMBAL_Q0
#define NEW_EKF_CHASSIS_Q_OFFSET        NEW_EKF_STATE_CHASSIS_Q0
#define NEW_EKF_GIMBAL_BIAS_OFFSET      NEW_EKF_STATE_GIMBAL_BIAS_X
#define NEW_EKF_CHASSIS_BIAS_OFFSET     NEW_EKF_STATE_CHASSIS_BIAS_X

#define NEW_EKF_INITIAL_Q_VARIANCE      100000.0f
#define NEW_EKF_INITIAL_BIAS_VARIANCE   100.0f
#define NEW_EKF_MAX_BIAS_VARIANCE       10000.0f
#define NEW_EKF_MIN_NORM_SQUARED        1.0e-12f
#define NEW_EKF_MIN_EULER_DENOMINATOR   1.0e-6f
#define NEW_EKF_MAX_BIAS_CORRECTION     1.0e-2f
#define NEW_EKF_DIVERGENCE_COUNT        50U
#define NEW_EKF_PI                      3.14159265358979323846f
#define NEW_EKF_TWO_PI                  (2.0f * NEW_EKF_PI)

static void NEW_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t* kf);
static void NEW_QuaternionEKF_SetH(KalmanFilter_t* kf);
static void NEW_QuaternionEKF_xhatUpdate(KalmanFilter_t* kf);
static void NEW_QuaternionEKF_Normalize(KalmanFilter_t* kf);

static float clampFloat(float value, float minimum, float maximum);
static float wrapAngleRad(float angleRad);
static uint8_t normalizeVector3(const float input[3], float output[3], float* norm);
static uint8_t normalizeQuaternion(float q[4]);
static void setGravityJacobian(float* hData, uint8_t rowOffset, uint8_t qOffset,
                               const float q[4]);
static void setQuaternionTransition(KalmanFilter_t* kf, uint8_t qOffset,
                                    const float gyro[3], float dt);
static void setQuaternionBiasJacobian(KalmanFilter_t* kf, uint8_t qOffset,
                                      uint8_t biasOffset, const float q[4], float dt);
static void calculateEncoderObservation(const NEW_QEKF_INS_t* ekf, const float gimbalQ[4],
                                        const float chassisQ[4], float* yawRad,
                                        float* pitchRad, float yawJacobian[14],
                                        float pitchJacobian[14]);
static void calculateMeasurement(const NEW_QEKF_INS_t* ekf, const float state[14], float measurement[8]);
static void updateStaticState(NEW_QEKF_INS_t* ekf, const NEW_QEKF_Input_t* input);
static void applyChassisStaticBiasUpdate(NEW_QEKF_INS_t* ekf, const float measuredGyro[3]);
static void copyFilterOutput(NEW_QEKF_INS_t* ekf);
static uint8_t filterAllocationSucceeded(const KalmanFilter_t* kf);
static void initializeCovariance(NEW_QEKF_INS_t* ekf);

void NEW_QuaternionEKF_ConfigDefault(NEW_QEKF_Config_t* config)
{
    if(config == NULL)
    {
        return;
    }
    //设定过程噪声，观测噪声，观测噪声需要实际采样获得
    memset(config, 0, sizeof(*config));
    config->Q1 = 10.0f;
    config->Q2 = 0.001f;

    config->R[NEW_EKF_MEAS_GIMBAL_ACCEL_X] = 10000000.0f;
    config->R[NEW_EKF_MEAS_GIMBAL_ACCEL_Y] = 10000000.0f;
    config->R[NEW_EKF_MEAS_GIMBAL_ACCEL_Z] = 10000000.0f;
    config->R[NEW_EKF_MEAS_CHASSIS_ACCEL_X] = 10000000.0f;
    config->R[NEW_EKF_MEAS_CHASSIS_ACCEL_Y] = 10000000.0f;
    config->R[NEW_EKF_MEAS_CHASSIS_ACCEL_Z] = 10000000.0f;
    config->R[NEW_EKF_MEAS_ENCODER_YAW] = 0.001f;
    config->R[NEW_EKF_MEAS_ENCODER_PITCH] = 0.001f;
    //?
    config->ChassisStaticBiasR[NEW_EKF_AXIS_X] = 0.0001f;
    config->ChassisStaticBiasR[NEW_EKF_AXIS_Y] = 0.0001f;
    config->ChassisStaticBiasR[NEW_EKF_AXIS_Z] = 0.0001f;

    config->ChiSquareTestThreshold = 15.507f;
    config->lambda = 1.0f;
    config->Gravity = 9.8f;
    //静止阈值判断
    config->StaticGyroThreshold = 0.3f;
    config->StaticAccelTolerance = 0.5f;
    config->StaticConfirmCount = 25U;
    //底盘和p轴的imu安装矩阵
    config->ChassisIMUToBodyQ[0] = 1.0f;
    config->PitchToGimbalIMUQ[0] = 1.0f;
}

uint8_t NEW_QuaternionEKF_Init(NEW_QEKF_INS_t* ekf, const NEW_QEKF_Config_t* config)
{
    NEW_QEKF_Config_t defaultConfig;
    //何意味啊？
    if(ekf == NULL)
    {
        return 0U;
    }

    NEW_QuaternionEKF_ConfigDefault(&defaultConfig);
    if(config == NULL)
    {
        config = &defaultConfig;
    }

    memset(ekf, 0, sizeof(*ekf));
    memcpy(&ekf->Config, config, sizeof(ekf->Config));

    if(!isfinite(ekf->Config.Q1) ||
       ekf->Config.Q1 <= 0.0f)
    {
        ekf->Config.Q1 = defaultConfig.Q1;
    }
    if(!isfinite(ekf->Config.Q2) ||
       ekf->Config.Q2 <= 0.0f)
    {
        ekf->Config.Q2 = defaultConfig.Q2;
    }
    for(uint8_t i = 0U; i < NEW_EKF_MEASUREMENT_SIZE; i++)
    {
        if(!isfinite(ekf->Config.R[i]) ||
           ekf->Config.R[i] <= 0.0f)
        {
            ekf->Config.R[i] = defaultConfig.R[i];
        }
    }
    for(uint8_t i = 0U; i < NEW_EKF_VECTOR_SIZE; i++)
    {
        if(!isfinite(ekf->Config.ChassisStaticBiasR[i]) ||
           ekf->Config.ChassisStaticBiasR[i] <= 0.0f)
        {
            ekf->Config.ChassisStaticBiasR[i] =
                defaultConfig.ChassisStaticBiasR[i];
        }
    }
    if(!isfinite(ekf->Config.lambda) || ekf->Config.lambda <= 0.0f ||
       ekf->Config.lambda > 1.0f)
    {
        ekf->Config.lambda = defaultConfig.lambda;
    }
    if(!isfinite(ekf->Config.ChiSquareTestThreshold) || ekf->Config.ChiSquareTestThreshold <= 0.0f)
    {
        ekf->Config.ChiSquareTestThreshold = defaultConfig.ChiSquareTestThreshold;
    }
    if(!isfinite(ekf->Config.Gravity) || ekf->Config.Gravity <= 0.0f)
    {
        ekf->Config.Gravity = defaultConfig.Gravity;
    }
    if(!isfinite(ekf->Config.StaticGyroThreshold) ||
       ekf->Config.StaticGyroThreshold <= 0.0f)
    {
        ekf->Config.StaticGyroThreshold =
            defaultConfig.StaticGyroThreshold;
    }
    if(!isfinite(ekf->Config.StaticAccelTolerance) ||
       ekf->Config.StaticAccelTolerance <= 0.0f)
    {
        ekf->Config.StaticAccelTolerance =
            defaultConfig.StaticAccelTolerance;
    }
    if(ekf->Config.StaticConfirmCount == 0U)
    {
        ekf->Config.StaticConfirmCount = defaultConfig.StaticConfirmCount;
    }
    if(!isfinite(ekf->Config.YawEncoderZero))
    {
        ekf->Config.YawEncoderZero = 0.0f;
    }
    if(!isfinite(ekf->Config.PitchEncoderZero))
    {
        ekf->Config.PitchEncoderZero = 0.0f;
    }

    if(!normalizeQuaternion(ekf->Config.ChassisIMUToBodyQ))
    {
        memset(ekf->Config.ChassisIMUToBodyQ, 0,
               sizeof(ekf->Config.ChassisIMUToBodyQ));
        ekf->Config.ChassisIMUToBodyQ[0] = 1.0f;
    }
    if(!normalizeQuaternion(ekf->Config.PitchToGimbalIMUQ))
    {
        memset(ekf->Config.PitchToGimbalIMUQ, 0,
               sizeof(ekf->Config.PitchToGimbalIMUQ));
        ekf->Config.PitchToGimbalIMUQ[0] = 1.0f;
    }

    Kalman_Filter_Init(&ekf->IMU_QuaternionEKF, NEW_EKF_STATE_SIZE, 0U, NEW_EKF_MEASUREMENT_SIZE);
    if(!filterAllocationSucceeded(&ekf->IMU_QuaternionEKF))
    {
        return 0U;
    }

    ekf->IMU_QuaternionEKF.User_Func1_f = NEW_QuaternionEKF_F_Linearization_P_Fading;
    ekf->IMU_QuaternionEKF.User_Func2_f = NEW_QuaternionEKF_SetH;
    ekf->IMU_QuaternionEKF.User_Func3_f = NEW_QuaternionEKF_xhatUpdate;
    ekf->IMU_QuaternionEKF.User_Func5_f = NEW_QuaternionEKF_Normalize;
    ekf->IMU_QuaternionEKF.SkipEq3 = 1U;
    ekf->IMU_QuaternionEKF.SkipEq4 = 1U;
    ekf->AdaptiveGainScale = 1.0f;
    ekf->Initialized = 1U;

    return NEW_QuaternionEKF_Reset(ekf, NULL, NULL);
}

uint8_t NEW_QuaternionEKF_Reset(NEW_QEKF_INS_t* ekf, const float gimbalQ[4], const float chassisQ[4])
{
    float initialGimbalQ[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float initialChassisQ[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    if(ekf == NULL || !ekf->Initialized)
    {
        return 0U;
    }

    if(gimbalQ != NULL)
    {
        memcpy(initialGimbalQ, gimbalQ, sizeof(initialGimbalQ));
    }
    if(chassisQ != NULL)
    {
        memcpy(initialChassisQ, chassisQ, sizeof(initialChassisQ));
    }
    if(!normalizeQuaternion(initialGimbalQ) || !normalizeQuaternion(initialChassisQ))
    {
        return 0U;
    }

    memset(ekf->IMU_QuaternionEKF.xhat_data, 0, sizeof(float) * NEW_EKF_STATE_SIZE);
    memcpy(&ekf->IMU_QuaternionEKF.xhat_data[NEW_EKF_GIMBAL_Q_OFFSET], initialGimbalQ,
           sizeof(initialGimbalQ));
    memcpy(&ekf->IMU_QuaternionEKF.xhat_data[NEW_EKF_CHASSIS_Q_OFFSET], initialChassisQ,
           sizeof(initialChassisQ));
    memcpy(ekf->IMU_QuaternionEKF.xhatminus_data, ekf->IMU_QuaternionEKF.xhat_data,
           sizeof(float) * NEW_EKF_STATE_SIZE);
    memcpy(ekf->IMU_QuaternionEKF.FilteredValue, ekf->IMU_QuaternionEKF.xhat_data,
           sizeof(float) * NEW_EKF_STATE_SIZE);

    initializeCovariance(ekf);
    ekf->ConvergeFlag = 0U;
    ekf->ChassisStaticFlag = 0U;
    ekf->StaticCount = 0U;
    ekf->ErrorCount = 0U;
    ekf->UpdateCount = 0U;
    ekf->ChiSquare = 0.0f;
    ekf->AdaptiveGainScale = 1.0f;
    copyFilterOutput(ekf);
    return 1U;
}

uint8_t NEW_QuaternionEKF_SetR(NEW_QEKF_INS_t* ekf, NEW_QEKF_MeasurementIndex_e measurement,
                                  float variance)
{
    if(ekf == NULL || !ekf->Initialized || (int32_t)measurement < 0 ||
       measurement >= NEW_EKF_MEASUREMENT_SIZE ||
       !isfinite(variance) || variance <= 0.0f)
    {
        return 0U;
    }

    ekf->Config.R[measurement] = variance;
    return 1U;
}

uint8_t NEW_QuaternionEKF_SetChassisStaticBiasR(NEW_QEKF_INS_t* ekf, NEW_QEKF_Axis_e axis, float variance)
{
    if(ekf == NULL || !ekf->Initialized || (int32_t)axis < 0 ||
       axis >= NEW_EKF_VECTOR_SIZE ||
       !isfinite(variance) || variance <= 0.0f)
    {
        return 0U;
    }

    ekf->Config.ChassisStaticBiasR[axis] = variance;
    return 1U;
}

uint8_t NEW_QuaternionEKF_Update(NEW_QEKF_INS_t* ekf, const NEW_QEKF_Input_t* input)
{   
    float gimbalAccelNorm;
    float chassisAccelNorm;

    if(ekf == NULL || input == NULL || !ekf->Initialized ||
       !isfinite(input->dt) || input->dt <= 0.0f ||
       !isfinite(input->YawEncoder) || !isfinite(input->PitchEncoder))
    {
        return 0U;
    }

    for(uint8_t i = 0U; i < NEW_EKF_VECTOR_SIZE; i++)
    {
        if(!isfinite(input->GimbalGyro[i]) || !isfinite(input->ChassisGyro[i]) ||
           !isfinite(input->GimbalAccel[i]) || !isfinite(input->ChassisAccel[i]))
        {
            return 0U;
        }
    }

    if(!normalizeVector3(input->GimbalAccel, ekf->GimbalAccel,
                          &gimbalAccelNorm) ||
       !normalizeVector3(input->ChassisAccel, ekf->ChassisAccel,
                          &chassisAccelNorm))
    {
        return 0U;
    }

    ekf->dt = input->dt;
    for(uint8_t i = 0U; i < NEW_EKF_VECTOR_SIZE; i++)
    {   //更新六轴输入状态
        ekf->GimbalGyro[i] = input->GimbalGyro[i] -
                                    ekf->GimbalGyroBias[i];
        ekf->ChassisGyro[i] = input->ChassisGyro[i] -
                                     ekf->ChassisGyroBias[i];
        ekf->IMU_QuaternionEKF.MeasuredVector[NEW_EKF_MEAS_GIMBAL_ACCEL_X + i] =
            ekf->GimbalAccel[i];
        ekf->IMU_QuaternionEKF.MeasuredVector[NEW_EKF_MEAS_CHASSIS_ACCEL_X + i] =
            ekf->ChassisAccel[i];
    }
    ekf->IMU_QuaternionEKF.MeasuredVector[NEW_EKF_MEAS_ENCODER_YAW] =
        wrapAngleRad(input->YawEncoder - ekf->Config.YawEncoderZero);
    ekf->IMU_QuaternionEKF.MeasuredVector[NEW_EKF_MEAS_ENCODER_PITCH] =
        wrapAngleRad(input->PitchEncoder - ekf->Config.PitchEncoderZero);
    //是否静止？
    updateStaticState(ekf, input);

    memset(ekf->IMU_QuaternionEKF.F_data, 0, sizeof(float) * NEW_EKF_STATE_SIZE * NEW_EKF_STATE_SIZE);
    for(uint8_t i = 0U; i < NEW_EKF_STATE_SIZE; i++)
    {
        ekf->IMU_QuaternionEKF.F_data[i * NEW_EKF_STATE_SIZE + i] = 1.0f;
    }
    setQuaternionTransition(&ekf->IMU_QuaternionEKF, NEW_EKF_GIMBAL_Q_OFFSET,
                              ekf->GimbalGyro, ekf->dt);
    setQuaternionTransition(&ekf->IMU_QuaternionEKF, NEW_EKF_CHASSIS_Q_OFFSET,
                              ekf->ChassisGyro, ekf->dt);
    
    memset(ekf->IMU_QuaternionEKF.Q_data, 0, sizeof(float) * NEW_EKF_STATE_SIZE * NEW_EKF_STATE_SIZE);
    for(uint8_t i = 0U; i < 8U; i++)
    {
        ekf->IMU_QuaternionEKF.Q_data[i * NEW_EKF_STATE_SIZE + i] =
            ekf->Config.Q1 * ekf->dt;
    }
    for(uint8_t i = 8U; i < NEW_EKF_STATE_SIZE; i++)
    {
        ekf->IMU_QuaternionEKF.Q_data[i * NEW_EKF_STATE_SIZE + i] =
            ekf->Config.Q2 * ekf->dt;
    }

    memset(ekf->IMU_QuaternionEKF.R_data, 0,
           sizeof(float) * NEW_EKF_MEASUREMENT_SIZE * NEW_EKF_MEASUREMENT_SIZE);
    for(uint8_t i = 0U; i < NEW_EKF_MEASUREMENT_SIZE; i++)
    {
        float variance = ekf->Config.R[i];
        if(!isfinite(variance) || variance <= 0.0f)
        {
            return 0U;
        }
        ekf->IMU_QuaternionEKF.R_data[i * NEW_EKF_MEASUREMENT_SIZE + i] = variance;
    }

    Kalman_Filter_Update(&ekf->IMU_QuaternionEKF);
    if(ekf->ChassisStaticFlag)
    {
        applyChassisStaticBiasUpdate(ekf, input->ChassisGyro);
    }

    NEW_QuaternionEKF_Normalize(&ekf->IMU_QuaternionEKF);
    memcpy(ekf->IMU_QuaternionEKF.FilteredValue, ekf->IMU_QuaternionEKF.xhat_data,
           sizeof(float) * NEW_EKF_STATE_SIZE);
    copyFilterOutput(ekf);
    ekf->UpdateCount++;
    return 1U;
}

static void NEW_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t* kf)
{
    NEW_QEKF_INS_t* ekf = (NEW_QEKF_INS_t*)kf;

    if(!normalizeQuaternion(&kf->xhatminus_data[NEW_EKF_GIMBAL_Q_OFFSET]))
    {
        memset(&kf->xhatminus_data[NEW_EKF_GIMBAL_Q_OFFSET], 0, sizeof(float) * 4U);
        kf->xhatminus_data[NEW_EKF_GIMBAL_Q_OFFSET] = 1.0f;
    }
    if(!normalizeQuaternion(&kf->xhatminus_data[NEW_EKF_CHASSIS_Q_OFFSET]))
    {
        memset(&kf->xhatminus_data[NEW_EKF_CHASSIS_Q_OFFSET], 0, sizeof(float) * 4U);
        kf->xhatminus_data[NEW_EKF_CHASSIS_Q_OFFSET] = 1.0f;
    }

    setQuaternionBiasJacobian(kf, NEW_EKF_GIMBAL_Q_OFFSET,
                                 NEW_EKF_GIMBAL_BIAS_OFFSET,
                                 &kf->xhatminus_data[NEW_EKF_GIMBAL_Q_OFFSET], ekf->dt);
    setQuaternionBiasJacobian(kf, NEW_EKF_CHASSIS_Q_OFFSET,
                                 NEW_EKF_CHASSIS_BIAS_OFFSET,
                                 &kf->xhatminus_data[NEW_EKF_CHASSIS_Q_OFFSET], ekf->dt);

    for(uint8_t i = NEW_EKF_GIMBAL_BIAS_OFFSET; i < NEW_EKF_STATE_SIZE; i++)
    {
        uint16_t diagonal = (uint16_t)i * NEW_EKF_STATE_SIZE + i;
        kf->P_data[diagonal] /= ekf->Config.lambda;
        if(kf->P_data[diagonal] > NEW_EKF_MAX_BIAS_VARIANCE)
        {
            kf->P_data[diagonal] = NEW_EKF_MAX_BIAS_VARIANCE;
        }
    }
}

static void NEW_QuaternionEKF_SetH(KalmanFilter_t* kf)
{
    NEW_QEKF_INS_t* ekf = (NEW_QEKF_INS_t*)kf;
    float yawJacobian[NEW_EKF_STATE_SIZE] = {0.0f};
    float pitchJacobian[NEW_EKF_STATE_SIZE] = {0.0f};
    float yawRad;
    float pitchRad;

    memset(kf->H_data, 0,
           sizeof(float) * NEW_EKF_MEASUREMENT_SIZE * NEW_EKF_STATE_SIZE);
    setGravityJacobian(kf->H_data, NEW_EKF_MEAS_GIMBAL_ACCEL_X,
                         NEW_EKF_GIMBAL_Q_OFFSET,
                         &kf->xhatminus_data[NEW_EKF_GIMBAL_Q_OFFSET]);
    setGravityJacobian(kf->H_data, NEW_EKF_MEAS_CHASSIS_ACCEL_X,
                         NEW_EKF_CHASSIS_Q_OFFSET,
                         &kf->xhatminus_data[NEW_EKF_CHASSIS_Q_OFFSET]);

    calculateEncoderObservation(ekf,
                                  &kf->xhatminus_data[NEW_EKF_GIMBAL_Q_OFFSET],
                                  &kf->xhatminus_data[NEW_EKF_CHASSIS_Q_OFFSET],
                                  &yawRad, &pitchRad, yawJacobian, pitchJacobian);
    memcpy(&kf->H_data[NEW_EKF_MEAS_ENCODER_YAW * NEW_EKF_STATE_SIZE],
           yawJacobian, sizeof(yawJacobian));
    memcpy(&kf->H_data[NEW_EKF_MEAS_ENCODER_PITCH * NEW_EKF_STATE_SIZE],
           pitchJacobian, sizeof(pitchJacobian));
}

static void NEW_QuaternionEKF_xhatUpdate(KalmanFilter_t* kf)
{
    NEW_QEKF_INS_t* ekf = (NEW_QEKF_INS_t*)kf;

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix);
    kf->temp_matrix1.numRows = kf->H.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);
    if(kf->MatStatus != ARM_MATH_SUCCESS)
    {
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof(float) * NEW_EKF_STATE_SIZE);
        memcpy(kf->P_data, kf->Pminus_data,
               sizeof(float) * NEW_EKF_STATE_SIZE * NEW_EKF_STATE_SIZE);
        kf->SkipEq5 = 1U;
        return;
    }

    calculateMeasurement(ekf, kf->xhatminus_data, ekf->PredictedMeasurement);
    for(uint8_t i = 0U; i < NEW_EKF_MEASUREMENT_SIZE; i++)
    {
        ekf->Innovation[i] = kf->z_data[i] - ekf->PredictedMeasurement[i];
    }
    ekf->Innovation[NEW_EKF_MEAS_ENCODER_YAW] =
        wrapAngleRad(ekf->Innovation[NEW_EKF_MEAS_ENCODER_YAW]);
    ekf->Innovation[NEW_EKF_MEAS_ENCODER_PITCH] =
        wrapAngleRad(ekf->Innovation[NEW_EKF_MEAS_ENCODER_PITCH]);

    memcpy(kf->temp_vector_data1, ekf->Innovation,
           sizeof(float) * NEW_EKF_MEASUREMENT_SIZE);
    kf->temp_vector1.numRows = NEW_EKF_MEASUREMENT_SIZE;
    kf->temp_vector1.numCols = 1U;
    kf->temp_matrix.numRows = NEW_EKF_MEASUREMENT_SIZE;
    kf->temp_matrix.numCols = 1U;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1,
                                    &kf->temp_matrix);

    ekf->ChiSquare = 0.0f;
    for(uint8_t i = 0U; i < NEW_EKF_MEASUREMENT_SIZE; i++)
    {
        ekf->ChiSquare += ekf->Innovation[i] * kf->temp_matrix_data[i];
    }

    if(ekf->ChiSquare < 0.5f * ekf->Config.ChiSquareTestThreshold)
    {
        ekf->ConvergeFlag = 1U;
    }

    if(ekf->ChiSquare > ekf->Config.ChiSquareTestThreshold && ekf->ConvergeFlag)
    {
        if(ekf->ChassisStaticFlag)
        {
            ekf->ErrorCount++;
        }
        else
        {
            ekf->ErrorCount = 0U;
        }

        if(ekf->ErrorCount > NEW_EKF_DIVERGENCE_COUNT)
        {
            ekf->ConvergeFlag = 0U;
            kf->SkipEq5 = 0U;
        }
        else
        {
            memcpy(kf->xhat_data, kf->xhatminus_data,
                   sizeof(float) * NEW_EKF_STATE_SIZE);
            memcpy(kf->P_data, kf->Pminus_data,
                   sizeof(float) * NEW_EKF_STATE_SIZE * NEW_EKF_STATE_SIZE);
            kf->SkipEq5 = 1U;
            return;
        }
    }
    else
    {
        if(ekf->ChiSquare > 0.1f * ekf->Config.ChiSquareTestThreshold && ekf->ConvergeFlag)
        {
            ekf->AdaptiveGainScale =
                (ekf->Config.ChiSquareTestThreshold - ekf->ChiSquare) /
                (0.9f * ekf->Config.ChiSquareTestThreshold);
            ekf->AdaptiveGainScale = clampFloat(ekf->AdaptiveGainScale, 0.0f, 1.0f);
        }
        else
        {
            ekf->AdaptiveGainScale = 1.0f;
        }
        ekf->ErrorCount = 0U;
        kf->SkipEq5 = 0U;
    }

    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    for(uint16_t i = 0U; i < NEW_EKF_STATE_SIZE * NEW_EKF_MEASUREMENT_SIZE; i++)
    {
        kf->K_data[i] *= ekf->AdaptiveGainScale;
    }

    kf->temp_vector.numRows = NEW_EKF_STATE_SIZE;
    kf->temp_vector.numCols = 1U;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector);

    if(ekf->ConvergeFlag)
    {
        float correctionLimit = NEW_EKF_MAX_BIAS_CORRECTION * ekf->dt;
        for(uint8_t i = NEW_EKF_GIMBAL_BIAS_OFFSET; i < NEW_EKF_STATE_SIZE; i++)
        {
            kf->temp_vector_data[i] = clampFloat(kf->temp_vector_data[i],
                                             -correctionLimit, correctionLimit);
        }
    }

    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
}

static void NEW_QuaternionEKF_Normalize(KalmanFilter_t* kf)
{
    if(!normalizeQuaternion(&kf->xhat_data[NEW_EKF_GIMBAL_Q_OFFSET]))
    {
        memset(&kf->xhat_data[NEW_EKF_GIMBAL_Q_OFFSET], 0, sizeof(float) * 4U);
        kf->xhat_data[NEW_EKF_GIMBAL_Q_OFFSET] = 1.0f;
    }
    if(!normalizeQuaternion(&kf->xhat_data[NEW_EKF_CHASSIS_Q_OFFSET]))
    {
        memset(&kf->xhat_data[NEW_EKF_CHASSIS_Q_OFFSET], 0, sizeof(float) * 4U);
        kf->xhat_data[NEW_EKF_CHASSIS_Q_OFFSET] = 1.0f;
    }
}

static float clampFloat(float value, float minimum, float maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

static float wrapAngleRad(float angleRad)
{
    while(angleRad > NEW_EKF_PI)
    {
        angleRad -= NEW_EKF_TWO_PI;
    }
    while(angleRad < -NEW_EKF_PI)
    {
        angleRad += NEW_EKF_TWO_PI;
    }
    return angleRad;
}

static uint8_t normalizeVector3(const float input[3], float output[3], float* norm)
{
    float normSquared = input[0] * input[0] + input[1] * input[1] + input[2] * input[2];
    if(!isfinite(normSquared) || normSquared < NEW_EKF_MIN_NORM_SQUARED)
    {
        return 0U;
    }

    *norm = sqrtf(normSquared);
    float inverseNorm = 1.0f / *norm;
    output[0] = input[0] * inverseNorm;
    output[1] = input[1] * inverseNorm;
    output[2] = input[2] * inverseNorm;
    return 1U;
}

static uint8_t normalizeQuaternion(float q[4])
{
    float norm;
    float normalizedQ[4];

    arm_quaternion_norm_f32(q, &norm, 1U);
    if(!isfinite(norm) || norm * norm < NEW_EKF_MIN_NORM_SQUARED)
    {
        return 0U;
    }

    arm_quaternion_normalize_f32(q, normalizedQ, 1U);
    memcpy(q, normalizedQ, sizeof(normalizedQ));
    return 1U;
}

static void setGravityJacobian(float* hData, uint8_t rowOffset, uint8_t qOffset,
                                 const float q[4])
{
    float* rowX = &hData[rowOffset * NEW_EKF_STATE_SIZE + qOffset];
    float* rowY = &hData[(rowOffset + 1U) * NEW_EKF_STATE_SIZE + qOffset];
    float* rowZ = &hData[(rowOffset + 2U) * NEW_EKF_STATE_SIZE + qOffset];

    rowX[0] = -2.0f * q[2];
    rowX[1] = 2.0f * q[3];
    rowX[2] = -2.0f * q[0];
    rowX[3] = 2.0f * q[1];

    rowY[0] = 2.0f * q[1];
    rowY[1] = 2.0f * q[0];
    rowY[2] = 2.0f * q[3];
    rowY[3] = 2.0f * q[2];

    rowZ[0] = 2.0f * q[0];
    rowZ[1] = -2.0f * q[1];
    rowZ[2] = -2.0f * q[2];
    rowZ[3] = 2.0f * q[3];
}

static void setQuaternionTransition(KalmanFilter_t* kf, uint8_t qOffset,
                                    const float gyro[3], float dt)
{
    float halfxdt = 0.5f * gyro[0] * dt;
    float halfydt = 0.5f * gyro[1] * dt;
    float halfzdt = 0.5f * gyro[2] * dt;
    float* f = kf->F_data;
    uint8_t n = NEW_EKF_STATE_SIZE;

    f[(qOffset + 0U) * n + qOffset + 1U] = -halfxdt;
    f[(qOffset + 0U) * n + qOffset + 2U] = -halfydt;
    f[(qOffset + 0U) * n + qOffset + 3U] = -halfzdt;
    f[(qOffset + 1U) * n + qOffset + 0U] = halfxdt;
    f[(qOffset + 1U) * n + qOffset + 2U] = halfzdt;
    f[(qOffset + 1U) * n + qOffset + 3U] = -halfydt;
    f[(qOffset + 2U) * n + qOffset + 0U] = halfydt;
    f[(qOffset + 2U) * n + qOffset + 1U] = -halfzdt;
    f[(qOffset + 2U) * n + qOffset + 3U] = halfxdt;
    f[(qOffset + 3U) * n + qOffset + 0U] = halfzdt;
    f[(qOffset + 3U) * n + qOffset + 1U] = halfydt;
    f[(qOffset + 3U) * n + qOffset + 2U] = -halfxdt;
}

static void setQuaternionBiasJacobian(KalmanFilter_t* kf, uint8_t qOffset,
                                        uint8_t biasOffset, const float q[4], float dt)
{
    float halfdt = 0.5f * dt;
    float* f = kf->F_data;
    uint8_t n = NEW_EKF_STATE_SIZE;

    f[(qOffset + 0U) * n + biasOffset + 0U] = q[1] * halfdt;
    f[(qOffset + 0U) * n + biasOffset + 1U] = q[2] * halfdt;
    f[(qOffset + 0U) * n + biasOffset + 2U] = q[3] * halfdt;
    f[(qOffset + 1U) * n + biasOffset + 0U] = -q[0] * halfdt;
    f[(qOffset + 1U) * n + biasOffset + 1U] = q[3] * halfdt;
    f[(qOffset + 1U) * n + biasOffset + 2U] = -q[2] * halfdt;
    f[(qOffset + 2U) * n + biasOffset + 0U] = -q[3] * halfdt;
    f[(qOffset + 2U) * n + biasOffset + 1U] = -q[0] * halfdt;
    f[(qOffset + 2U) * n + biasOffset + 2U] = q[1] * halfdt;
    f[(qOffset + 3U) * n + biasOffset + 0U] = q[2] * halfdt;
    f[(qOffset + 3U) * n + biasOffset + 1U] = -q[1] * halfdt;
    f[(qOffset + 3U) * n + biasOffset + 2U] = -q[0] * halfdt;
}

static void calculateEncoderObservation(const NEW_QEKF_INS_t* ekf, const float gimbalQ[4],
                                        const float chassisQ[4], float* yawRad,
                                        float* pitchRad, float yawJacobian[14],
                                        float pitchJacobian[14])
{
    static const float quaternionBasis[4][4] =
    {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    float bodyToChassisIMUQ[4];
    float bodyQ[4];
    float navigationToBodyQ[4];
    float bodyToGimbalIMUQ[4];
    float jointQ[4];
    float yawDerivativeQ[4];
    float pitchDerivativeQ[4];
    float yawNumerator;
    float yawDenominator;
    float yawDenominatorSquared;
    float pitchSine;
    float pitchDenominator;

    /* q_WB = q_WC * conj(q_BC), q_BP = conj(q_WB) * q_WG * q_GP. */
    /* The ideal two-axis mechanism satisfies q_BP = Rz(yaw) * Ry(pitch). */
    arm_quaternion_conjugate_f32(ekf->Config.ChassisIMUToBodyQ,
                                 bodyToChassisIMUQ, 1U);
    arm_quaternion_product_single_f32(chassisQ, bodyToChassisIMUQ, bodyQ);
    arm_quaternion_conjugate_f32(bodyQ, navigationToBodyQ, 1U);
    arm_quaternion_product_single_f32(navigationToBodyQ, gimbalQ,
                                      bodyToGimbalIMUQ);
    arm_quaternion_product_single_f32(bodyToGimbalIMUQ,
                                      ekf->Config.PitchToGimbalIMUQ, jointQ);

    yawNumerator = 2.0f * (jointQ[0] * jointQ[3] + jointQ[1] * jointQ[2]);
    yawDenominator = 1.0f - 2.0f * (jointQ[2] * jointQ[2] +
                                     jointQ[3] * jointQ[3]);
    pitchSine = clampFloat(2.0f * (jointQ[0] * jointQ[2] -
                                 jointQ[3] * jointQ[1]), -1.0f, 1.0f);
    *yawRad = atan2f(yawNumerator, yawDenominator);
    *pitchRad = asinf(pitchSine);

    if(yawJacobian == NULL || pitchJacobian == NULL)
    {
        return;
    }

    yawDenominatorSquared = yawNumerator * yawNumerator +
                              yawDenominator * yawDenominator;
    if(yawDenominatorSquared < NEW_EKF_MIN_EULER_DENOMINATOR)
    {
        yawDenominatorSquared = NEW_EKF_MIN_EULER_DENOMINATOR;
    }

    float dYawNumerator[4] =
    {
        2.0f * jointQ[3], 2.0f * jointQ[2],
        2.0f * jointQ[1], 2.0f * jointQ[0]
    };
    float dYawDenominator[4] =
    {
        0.0f, 0.0f, -4.0f * jointQ[2], -4.0f * jointQ[3]
    };
    for(uint8_t i = 0U; i < 4U; i++)
    {
        yawDerivativeQ[i] =
            (yawDenominator * dYawNumerator[i] -
             yawNumerator * dYawDenominator[i]) / yawDenominatorSquared;
    }

    pitchDenominator = sqrtf(clampFloat(1.0f - pitchSine * pitchSine,
                                     NEW_EKF_MIN_EULER_DENOMINATOR, 1.0f));
    pitchDerivativeQ[0] = 2.0f * jointQ[2] / pitchDenominator;
    pitchDerivativeQ[1] = -2.0f * jointQ[3] / pitchDenominator;
    pitchDerivativeQ[2] = 2.0f * jointQ[0] / pitchDenominator;
    pitchDerivativeQ[3] = -2.0f * jointQ[1] / pitchDenominator;

    for(uint8_t stateComponent = 0U; stateComponent < 4U; stateComponent++)
    {
        float bodyDelta[4];
        float navigationToBodyDelta[4];
        float firstProduct[4];
        float jointDelta[4];

        arm_quaternion_product_single_f32(quaternionBasis[stateComponent],
                                          bodyToChassisIMUQ, bodyDelta);
        arm_quaternion_conjugate_f32(bodyDelta, navigationToBodyDelta, 1U);
        arm_quaternion_product_single_f32(navigationToBodyDelta, gimbalQ,
                                          firstProduct);
        arm_quaternion_product_single_f32(firstProduct,
                                          ekf->Config.PitchToGimbalIMUQ,
                                          jointDelta);

        for(uint8_t outputComponent = 0U; outputComponent < 4U; outputComponent++)
        {
            yawJacobian[NEW_EKF_CHASSIS_Q_OFFSET + stateComponent] +=
                yawDerivativeQ[outputComponent] * jointDelta[outputComponent];
            pitchJacobian[NEW_EKF_CHASSIS_Q_OFFSET + stateComponent] +=
                pitchDerivativeQ[outputComponent] * jointDelta[outputComponent];
        }

        arm_quaternion_product_single_f32(navigationToBodyQ,
                                          quaternionBasis[stateComponent],
                                          firstProduct);
        arm_quaternion_product_single_f32(firstProduct,
                                          ekf->Config.PitchToGimbalIMUQ,
                                          jointDelta);
        for(uint8_t outputComponent = 0U; outputComponent < 4U; outputComponent++)
        {
            yawJacobian[NEW_EKF_GIMBAL_Q_OFFSET + stateComponent] +=
                yawDerivativeQ[outputComponent] * jointDelta[outputComponent];
            pitchJacobian[NEW_EKF_GIMBAL_Q_OFFSET + stateComponent] +=
                pitchDerivativeQ[outputComponent] * jointDelta[outputComponent];
        }
    }
}

static void calculateMeasurement(const NEW_QEKF_INS_t* ekf, const float state[14], float measurement[8])
{
    float rotation[9];

    arm_quaternion2rotation_f32(&state[NEW_EKF_GIMBAL_Q_OFFSET], rotation, 1U);
    memcpy(&measurement[NEW_EKF_MEAS_GIMBAL_ACCEL_X], &rotation[6], sizeof(float) * 3U);
    arm_quaternion2rotation_f32(&state[NEW_EKF_CHASSIS_Q_OFFSET], rotation, 1U);
    memcpy(&measurement[NEW_EKF_MEAS_CHASSIS_ACCEL_X], &rotation[6], sizeof(float) * 3U);
    calculateEncoderObservation(ekf,
                                  &state[NEW_EKF_GIMBAL_Q_OFFSET],
                                  &state[NEW_EKF_CHASSIS_Q_OFFSET],
                                  &measurement[NEW_EKF_MEAS_ENCODER_YAW],
                                  &measurement[NEW_EKF_MEAS_ENCODER_PITCH],
                                  NULL, NULL);
}

static void updateStaticState(NEW_QEKF_INS_t* ekf, const NEW_QEKF_Input_t* input)
{
    float correctedGyro[3];
    float gyroNormSquared = 0.0f;
    float accelNormSquared = 0.0f;

    for(uint8_t i = 0U; i < 3U; i++)
    {
        correctedGyro[i] = input->ChassisGyro[i] -
                            ekf->ChassisGyroBias[i];
        gyroNormSquared += correctedGyro[i] * correctedGyro[i];
        accelNormSquared += input->ChassisAccel[i] *
                              input->ChassisAccel[i];
    }

    float gyroNorm = sqrtf(gyroNormSquared);
    float accelNorm = sqrtf(accelNormSquared);
    uint8_t staticCandidate =
        gyroNorm < ekf->Config.StaticGyroThreshold &&
        fabsf(accelNorm - ekf->Config.Gravity) <
            ekf->Config.StaticAccelTolerance;

    if(staticCandidate)
    {
        if(ekf->StaticCount < ekf->Config.StaticConfirmCount)
        {
            ekf->StaticCount++;
        }
        if(ekf->StaticCount >= ekf->Config.StaticConfirmCount)
        {
            ekf->ChassisStaticFlag = 1U;
        }
    }
    else
    {
        ekf->StaticCount = 0U;
        ekf->ChassisStaticFlag = 0U;
    }
}

static void applyChassisStaticBiasUpdate(NEW_QEKF_INS_t* ekf, const float measuredGyro[3])
{
    KalmanFilter_t* kf = &ekf->IMU_QuaternionEKF;
    float covarianceRow[NEW_EKF_STATE_SIZE];
    float gain[NEW_EKF_STATE_SIZE];

    for(uint8_t axis = 0U; axis < 3U; axis++)
    {
        uint8_t biasIndex = NEW_EKF_CHASSIS_BIAS_OFFSET + axis;
        float variance = ekf->Config.ChassisStaticBiasR[axis];
        float innovation = measuredGyro[axis] - kf->xhat_data[biasIndex];
        float innovationVariance =
            kf->P_data[biasIndex * NEW_EKF_STATE_SIZE + biasIndex] + variance;

        if(!isfinite(variance) || variance <= 0.0f ||
           !isfinite(innovationVariance) || innovationVariance <= 0.0f)
        {
            continue;
        }

        memcpy(covarianceRow,
               &kf->P_data[biasIndex * NEW_EKF_STATE_SIZE],
               sizeof(covarianceRow));
        for(uint8_t row = 0U; row < NEW_EKF_STATE_SIZE; row++)
        {
            gain[row] = kf->P_data[row * NEW_EKF_STATE_SIZE + biasIndex] /
                        innovationVariance;
            kf->xhat_data[row] += gain[row] * innovation;
        }
        for(uint8_t row = 0U; row < NEW_EKF_STATE_SIZE; row++)
        {
            for(uint8_t column = 0U; column < NEW_EKF_STATE_SIZE; column++)
            {
                kf->P_data[row * NEW_EKF_STATE_SIZE + column] -=
                    gain[row] * covarianceRow[column];
            }
        }
    }

    for(uint8_t row = 0U; row < NEW_EKF_STATE_SIZE; row++)
    {
        for(uint8_t column = row + 1U; column < NEW_EKF_STATE_SIZE; column++)
        {
            float symmetricValue = 0.5f *
                (kf->P_data[row * NEW_EKF_STATE_SIZE + column] +
                 kf->P_data[column * NEW_EKF_STATE_SIZE + row]);
            kf->P_data[row * NEW_EKF_STATE_SIZE + column] = symmetricValue;
            kf->P_data[column * NEW_EKF_STATE_SIZE + row] = symmetricValue;
        }
    }
}

static void copyFilterOutput(NEW_QEKF_INS_t* ekf)
{
    memcpy(ekf->GimbalQ, &ekf->IMU_QuaternionEKF.xhat_data[NEW_EKF_GIMBAL_Q_OFFSET],
           sizeof(ekf->GimbalQ));
    memcpy(ekf->ChassisQ, &ekf->IMU_QuaternionEKF.xhat_data[NEW_EKF_CHASSIS_Q_OFFSET],
           sizeof(ekf->ChassisQ));
    memcpy(ekf->GimbalGyroBias,
           &ekf->IMU_QuaternionEKF.xhat_data[NEW_EKF_GIMBAL_BIAS_OFFSET],
           sizeof(ekf->GimbalGyroBias));
    memcpy(ekf->ChassisGyroBias,
           &ekf->IMU_QuaternionEKF.xhat_data[NEW_EKF_CHASSIS_BIAS_OFFSET],
           sizeof(ekf->ChassisGyroBias));
}

static uint8_t filterAllocationSucceeded(const KalmanFilter_t* kf)
{
    return kf->FilteredValue != NULL && kf->MeasuredVector != NULL &&
           kf->MeasurementMap != NULL && kf->MeasurementDegree != NULL &&
           kf->MatR_DiagonalElements != NULL && kf->StateMinVariance != NULL &&
           kf->temp != NULL && kf->xhat_data != NULL && kf->xhatminus_data != NULL &&
           kf->z_data != NULL &&
           kf->P_data != NULL && kf->Pminus_data != NULL &&
           kf->F_data != NULL && kf->FT_data != NULL &&
           kf->H_data != NULL && kf->HT_data != NULL &&
           kf->Q_data != NULL && kf->R_data != NULL && kf->K_data != NULL &&
           kf->S_data != NULL && kf->temp_matrix_data != NULL &&
           kf->temp_matrix_data1 != NULL && kf->temp_vector_data != NULL &&
           kf->temp_vector_data1 != NULL;
}

static void initializeCovariance(NEW_QEKF_INS_t* ekf)
{
    memset(ekf->IMU_QuaternionEKF.P_data, 0,
           sizeof(float) * NEW_EKF_STATE_SIZE * NEW_EKF_STATE_SIZE);
    memset(ekf->IMU_QuaternionEKF.Pminus_data, 0,
           sizeof(float) * NEW_EKF_STATE_SIZE * NEW_EKF_STATE_SIZE);
    for(uint8_t i = 0U; i < NEW_EKF_STATE_SIZE; i++)
    {
        float variance = i < NEW_EKF_GIMBAL_BIAS_OFFSET ?
                         NEW_EKF_INITIAL_Q_VARIANCE : NEW_EKF_INITIAL_BIAS_VARIANCE;
        ekf->IMU_QuaternionEKF.P_data[i * NEW_EKF_STATE_SIZE + i] = variance;
        ekf->IMU_QuaternionEKF.Pminus_data[i * NEW_EKF_STATE_SIZE + i] = variance;
    }
}

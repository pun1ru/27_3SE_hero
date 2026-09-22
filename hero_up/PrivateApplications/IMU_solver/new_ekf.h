/**
 ******************************************************************************
 * @file    new_ekf.h
 * @brief   Coupled gimbal/chassis quaternion EKF interface
 ******************************************************************************
 */
#ifndef _NEW_EKF_H_
#define _NEW_EKF_H_

#include "kalman_filter.h"
#include <stdint.h>

#define NEW_EKF_STATE_SIZE              14U
#define NEW_EKF_MEASUREMENT_SIZE         8U
#define NEW_EKF_QUATERNION_SIZE          4U
#define NEW_EKF_VECTOR_SIZE              3U

typedef enum
{
    NEW_EKF_STATE_GIMBAL_Q0 = 0,
    NEW_EKF_STATE_GIMBAL_Q1,
    NEW_EKF_STATE_GIMBAL_Q2,
    NEW_EKF_STATE_GIMBAL_Q3,
    NEW_EKF_STATE_CHASSIS_Q0,
    NEW_EKF_STATE_CHASSIS_Q1,
    NEW_EKF_STATE_CHASSIS_Q2,
    NEW_EKF_STATE_CHASSIS_Q3,
    NEW_EKF_STATE_GIMBAL_BIAS_X,
    NEW_EKF_STATE_GIMBAL_BIAS_Y,
    NEW_EKF_STATE_GIMBAL_BIAS_Z,
    NEW_EKF_STATE_CHASSIS_BIAS_X,
    NEW_EKF_STATE_CHASSIS_BIAS_Y,
    NEW_EKF_STATE_CHASSIS_BIAS_Z
} NEW_QEKF_StateIndex_e;

typedef enum
{
    NEW_EKF_MEAS_GIMBAL_ACCEL_X = 0,
    NEW_EKF_MEAS_GIMBAL_ACCEL_Y,
    NEW_EKF_MEAS_GIMBAL_ACCEL_Z,
    NEW_EKF_MEAS_CHASSIS_ACCEL_X,
    NEW_EKF_MEAS_CHASSIS_ACCEL_Y,
    NEW_EKF_MEAS_CHASSIS_ACCEL_Z,
    NEW_EKF_MEAS_ENCODER_YAW,
    NEW_EKF_MEAS_ENCODER_PITCH
} NEW_QEKF_MeasurementIndex_e;

typedef enum
{
    NEW_EKF_AXIS_X = 0,
    NEW_EKF_AXIS_Y,
    NEW_EKF_AXIS_Z
} NEW_QEKF_Axis_e;

typedef struct
{
    float Q1;
    float Q2;
    /* First six entries are normalized-acceleration variances; last two are rad^2. */
    float R[NEW_EKF_MEASUREMENT_SIZE];
    /* Conditional z = chassis gyro, h(x) = chassis gyro bias variances. */
    float ChassisStaticBiasR[NEW_EKF_VECTOR_SIZE];

    float ChiSquareTestThreshold;
    float lambda;
    float Gravity;
    float StaticGyroThreshold;
    float StaticAccelTolerance;
    uint16_t StaticConfirmCount;

    float YawEncoderZero;
    float PitchEncoderZero;

    /*
     * Quaternion convention is [w, x, y, z]. State quaternions rotate vectors
     * from each IMU frame to the navigation frame.
     *
     * ChassisIMUToBodyQ rotates chassis-IMU coordinates into body
     * coordinates. PitchToGimbalIMUQ rotates pitch-frame coordinates into
     * gimbal-IMU coordinates. Both default to identity. Therefore, at zero yaw
     * and pitch, the yaw and pitch frames initially coincide with the body frame.
     */
    float ChassisIMUToBodyQ[NEW_EKF_QUATERNION_SIZE];
    float PitchToGimbalIMUQ[NEW_EKF_QUATERNION_SIZE];
} NEW_QEKF_Config_t;

typedef struct
{
    float GimbalGyro[NEW_EKF_VECTOR_SIZE];
    float ChassisGyro[NEW_EKF_VECTOR_SIZE];
    float GimbalAccel[NEW_EKF_VECTOR_SIZE];
    float ChassisAccel[NEW_EKF_VECTOR_SIZE];
    float YawEncoder;
    float PitchEncoder;
    float dt;
} NEW_QEKF_Input_t;

typedef struct
{
    /* Must remain first: Kalman callbacks recover the owner from this member. */
    KalmanFilter_t IMU_QuaternionEKF;

    NEW_QEKF_Config_t Config;
    uint8_t Initialized;
    uint8_t ConvergeFlag;
    uint8_t ChassisStaticFlag;
    uint16_t StaticCount;
    uint32_t ErrorCount;
    uint32_t UpdateCount;

    float GimbalQ[NEW_EKF_QUATERNION_SIZE];
    float ChassisQ[NEW_EKF_QUATERNION_SIZE];
    float GimbalGyroBias[NEW_EKF_VECTOR_SIZE];
    float ChassisGyroBias[NEW_EKF_VECTOR_SIZE];
    float GimbalGyro[NEW_EKF_VECTOR_SIZE];
    float ChassisGyro[NEW_EKF_VECTOR_SIZE];
    float GimbalAccel[NEW_EKF_VECTOR_SIZE];
    float ChassisAccel[NEW_EKF_VECTOR_SIZE];
    float PredictedMeasurement[NEW_EKF_MEASUREMENT_SIZE];
    float Innovation[NEW_EKF_MEASUREMENT_SIZE];

    float dt;
    float ChiSquare;
    float AdaptiveGainScale;
} NEW_QEKF_INS_t;

/**
 * @brief   Fill a configuration with usable defaults based on ekf_quaternion.
 * @param   config Configuration destination.
 * @retval  void
 */
void NEW_QuaternionEKF_ConfigDefault(NEW_QEKF_Config_t* config);

/**
 * @brief   Initialize the coupled 14-state, 8-measurement EKF.
 * @param   ekf Filter instance. Initialize each instance only once.
 * @param   config Configuration, or NULL to use NEW_QuaternionEKF_ConfigDefault().
 * @retval  1 on success, 0 on invalid arguments or allocation failure.
 */
uint8_t NEW_QuaternionEKF_Init(NEW_QEKF_INS_t* ekf, const NEW_QEKF_Config_t* config);

/**
 * @brief   Reset both attitude states and all gyro-bias states.
 * @param   ekf Initialized filter instance.
 * @param   GimbalQ Gimbal-IMU-to-navigation quaternion, or NULL for identity.
 * @param   ChassisQ Chassis-IMU-to-navigation quaternion, or NULL for identity.
 * @retval  1 on success, 0 on invalid arguments.
 */
uint8_t NEW_QuaternionEKF_Reset(NEW_QEKF_INS_t* ekf, const float gimbalQ[NEW_EKF_QUATERNION_SIZE],
                                const float chassisQ[NEW_EKF_QUATERNION_SIZE]);

/**
 * @brief   Set one diagonal entry of the 8-dimensional measurement R matrix.
 * @param   ekf Initialized filter instance.
 * @param   measurement Measurement whose variance is being configured.
 * @param   variance Positive measurement variance.
 * @retval  1 on success, 0 on invalid arguments.
 */
uint8_t NEW_QuaternionEKF_SetR(NEW_QEKF_INS_t* ekf, NEW_QEKF_MeasurementIndex_e measurement,
                                  float variance);

/**
 * @brief   Set one chassis-static gyro-bias pseudo-measurement variance.
 * @param   ekf Initialized filter instance.
 * @param   axis Chassis gyro axis.
 * @param   variance Positive measurement variance in (rad/s)^2.
 * @retval  1 on success, 0 on invalid arguments.
 */
uint8_t NEW_QuaternionEKF_SetChassisStaticBiasR(NEW_QEKF_INS_t* ekf, NEW_QEKF_Axis_e axis, float variance);

/**
 * @brief   Run one prediction and measurement update.
 * @param   ekf Initialized filter instance.
 * @param   input Two IMU samples, encoder angles, and update period.
 * @retval  1 when the update was executed, 0 for invalid input.
 * @note    Gyroscope units are rad/s, acceleration units are m/s^2, encoder
 *          units are rad, and dt is seconds.
 */
uint8_t NEW_QuaternionEKF_Update(NEW_QEKF_INS_t* ekf, const NEW_QEKF_Input_t* input);

#endif

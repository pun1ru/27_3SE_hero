#ifndef _SHOOT_CONTROL_H_
#define _SHOOT_CONTROL_H_

#include "stdint.h"
#include "pid.h"
#include "algorism.h"

#define STIR_PRESET_ANGLE +10.0f
#define CHASSIS_MOTOR_FRONTFEED_RATIO 1500
#define STIR_CAUTION_SPEED 1.0f
#define STIR_MAX_SPEED 100
#define STIR_MAX_SPEED_LOW 100
#define SHOOT_FINISH 0
#define SHOOT_BACK 1
#define SHOOT_READY 2
#define SHOOT_PUSH 3
#define SHOOT_WAIT 5
#define SHOOT_DURING 4

#ifdef MATCH_MODE
#define INITIAL_FRIC_SPEED 4700
#else
#define INITIAL_FRIC_SPEED 4700
#endif
#define TARGET_BULLET_SPEED 15.10

/**
 * @brief 发射控制相关结构体
 */
typedef struct
{
    struct
    {
        float fric_speed_rpm[6];
        float stir_speed_rps;
        float stir_angle_d;
        float stir_target_pos;
        float stir_target_pos_rad;
        float stir_target_vol;
        float stir_all_target_pos_d;
        float stir_all_target_pos_rad;
        int shoot_flag;
        int shoot_cnt;
    }ShootTargetInput;

    struct
    {
        PIDStruct fric_speed_pid[6];
        int16_t fric_target_output[6];
        float stir_preset_angle;
        float stir_angular_velocity_dps;
    }ShootMotorControl;

    struct
    {
        uint8_t stir_block_flag;
        uint8_t stir_reset_flag;
        uint8_t stir_enableflag_detect;
        uint8_t stir_enableflag_desire;
        uint16_t shoot_count;
        float stir_real_angle;
        float stir_real_angle_rad;
        float stir_real_angle_d;
        float stir_angle_last;
        float stir_angle_cur;
        float stir_all_angle_d;
        int quan_shu_r;
    }ShootEstimate;
}ShootControl;

/* API */
void ShootInputUpdate(void);
void ShootEstimateUpdate(void);
void ShootControlUpdate(void);
uint16_t CRC16_Modbus(uint8_t *data, uint16_t length);

extern ShootControl shootControl;
extern const ShootControl* _shootControl;

/* 射击相关共享变量 */
extern float current_fric_speed;
extern float default_fric_speed;
extern float front_fric_speed;
extern float back_fric_speed;
extern float targetspeed[30];
extern float predict_speed0;
extern float mardio_speed;
extern int16_t fric_speed_left_target;
extern int16_t fric_speed_right_target;
extern int16_t fric_speed_up_target;
extern int16_t fric_speed_left_target1;
extern int16_t fric_speed_right_target1;
extern int16_t fric_speed_up_target1;
extern leastSquareLinear bulletSpeedAdaptation;

#endif

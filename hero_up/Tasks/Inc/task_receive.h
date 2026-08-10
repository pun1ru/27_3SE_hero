#ifndef TASK_RECEIVE_H_
#define TASK_RECEIVE_H_

#include <stdint.h>

#include "cmsis_compiler.h"

#define EVENT_GROUP_BIT_ERROR (1UL << 0UL)
#define EVENT_GROUP_BIT_DT7 (1UL << 1UL)
#define EVENT_GROUP_BIT_VT3 (1UL << 2UL)

typedef enum {
    ERROR_RECEIVE = 0,
    DT7,
    VT13
} RemoteSourceEnum;

typedef struct {
    uint8_t remote_source;
    __PACKED_STRUCT {
        unsigned switch_L1 : 2;
        unsigned switch_R1 : 2;
        unsigned reserved : 4;
    }
    Switch;
    __PACKED_STRUCT {
        unsigned level_key_W : 1;
        unsigned level_key_A : 1;
        unsigned level_key_S : 1;
        unsigned level_key_D : 1;
        unsigned level_key_SHIFT : 1;
        unsigned level_key_CTRL : 1;
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
    }
    PCKeyBoard;
    struct {
        float ch0;
        float ch1;
        float ch2;
        float ch3;
        float ch4;
    } RelativeCH;
    struct {
        uint8_t mouse_left;
        uint8_t mouse_right;
        int16_t mouse_speed_x;
        int16_t mouse_speed_y;
        int16_t mouse_speed_z;
    } PCMouse;
} NormRemoteCmd;

typedef struct {
    uint16_t frame_counter;
    int16_t torque_current_real;
    uint16_t mechanical_angle;
    int16_t mechanical_speed_rpm;
    uint8_t motor_temperature_d;
} DJIGMotorRec;

typedef struct {
    uint16_t frame_counter;
    uint32_t timestamp;
    int state;
    int p_int;
    int v_int;
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
} DMJ4310MotorRec;

#define HEAT_MAX (5000 - 1)
#define HEAT_MIN 500
#define HEAT_MID 3500
#define IMU_TARGET_TEMPERATURE 40

#define ONBOARD_EKF_SOLVE

typedef struct {
    float pitch_d;
    float yaw_d;
    float roll_d;
    float pitch_radps;
    float roll_radps;
    float yaw_radps;
} Pose;

extern const DJIGMotorRec *const _fricMotorRec;
extern const DJIGMotorRec *const _pitchMotorRec;
extern const DMJ4310MotorRec *const _DMyawMotorRec;
extern const NormRemoteCmd *const _normRemoteCmd;
extern const Pose *const _gimbalPose;
void RemoteRecInitialize(void);
void PeripheralRecEnable(void);
void RemoteRecRestart(void);
void RemoteRecTask(void *argument);
void IMUTask(void *argument);
void UpperCommRecHandler(uint8_t *rec_buf, uint32_t size);
void UpperPCCommTask(void *argument);
void BulletSpeedReceive(void);

#endif

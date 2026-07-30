#ifndef DM_DRIVER_H_
#define DM_DRIVER_H_

#include "stdint.h"

#include "CAN_driver.h"
#include "device_define.h"

typedef struct
{
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
}DMMotorRec;

/// @brief MIT模式结构体
typedef struct {
	/**
	* \brief pd parameters
	*/
	float kp, kd;
	
	/**
	* \brief output
	*/
	float angular_velocity_radps; //目标角速度
	float target_pos_output; //目标角度
	float target_toq_output; //目标力矩
	
}MITStruct;

/// @brief MIT控制指令参数（CAN帧下发用）
typedef struct {
	float p;    // 目标位置 (rad)
	float v;    // 目标速度 (rad/s)
	float Kp;   // 位置刚度
	float Kd;   // 速度阻尼
	float Tff;  // 前馈力矩 (Nm)
} MIT_Ctrl_t;

void MIT_Clear(MIT_Ctrl_t *mit);
void MIT_SetParam(MIT_Ctrl_t *mit, float p, float v, float Kp, float Kd, float Tff);

/** Function Declaration **/
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

void DM_MITControl(float _pos, float _vel, float _KP, float _KD, float _torq, uint8_t *adata);
void DM_MITControl_Send(FDCAN_HandleTypeDef* hcan, uint16_t id,float _pos, float _vel, float _KP, float _KD, float _torq);

void DM_MixControl(float _pos, float _vel, uint8_t *adata);

void DM_SpeedControl(float _vel, uint8_t *adata);

void DM_Enable(uint8_t *adata);

void DM_Disable(uint8_t *adata);

void clear_error(FDCAN_HandleTypeDef* hcan, uint16_t id);

void ctrl_motor(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
void ctrl_motor2(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);

void start_motor(FDCAN_HandleTypeDef* hcan, uint16_t id);
void lock_motor(FDCAN_HandleTypeDef* hcan, uint16_t id);

#endif
 

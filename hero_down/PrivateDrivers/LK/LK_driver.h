#ifndef __LK_DRIVER_H_
#define __LK_DRIVER_H_

#include "stdint.h"

#define LK_STDID 0x140

typedef struct
{
    int8_t temperature;
    int16_t iq;
    float speed_rps;
    uint16_t encoder;
    uint8_t frame_counter;
}LKMotorRec;

/** Function Declaration **/
void LK_pidRead(uint8_t* adata);
void LK_pidWrite_RAM(uint8_t* adata, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
void LK_pidWrite_ROM(uint8_t* adata, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki);
void LK_accelRead(uint8_t* adata);
void LK_accelWrite(uint8_t* adata, int32_t accel);
void LK_encoderRead(uint8_t* adata);
void LK_encoderSet(uint8_t* adata, uint16_t encoder_offset);
void LK_MultiLoop_angleRead(uint8_t* adata);
void LK_SingleLoop_angleRead(uint8_t* adata);
void LK_stateRead_1(uint8_t* adata);
void LK_stateRead_2(uint8_t* adata);
void LK_stateRead_3(uint8_t* adata);
void LK_errorClear(uint8_t* adata);
void LK_Motor_off(uint8_t* adata);
void LK_Motor_stop(uint8_t* adata);
void LK_Motor_run(uint8_t* adata);
void LK_iqControl(uint8_t* adata, int32_t iq_control);
void LK_powerControl(uint8_t* adata, int32_t power_control);
void LK_speedControl(uint8_t* adata, int32_t speed_control);
void LK_MultiLoop_angleControl(uint8_t* adata, int32_t angle_control);
void LK_MultiLoop_angleControl_limited(uint8_t* adata, uint16_t max_speed, int32_t angle_control);
void LK_SingleLoop_angleControl(uint8_t* adata, uint8_t spin_dir, uint16_t angle_control);
void LK_SingleLoop_angleControl_limited(uint8_t* adata, uint8_t spin_dir, uint16_t max_speed, uint32_t angle_control);
void LK_Increment_angleControl(uint8_t* adata, int32_t angle_increment);
void LK_Increment_angleControl_limited(uint8_t* adata, uint16_t max_speed, int32_t angle_increment);
void LK_Multi_iqControl(uint8_t* adata, int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4);
void LK_Multi_set_RAM(uint8_t* adata,int32_t motoAngle);
void LK_Multi_set_ROM_noAdvise(uint8_t* adata);
#endif

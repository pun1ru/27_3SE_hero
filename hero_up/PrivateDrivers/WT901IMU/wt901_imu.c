// 维特智能9轴陀螺仪
#include "wt901_imu.h"
#include "string.h"

// 用于临时存储传输来的未处理数据
Angle_t stcAngle;
Gyro_t stcGyro;

/**
 * @brief  处理接收到的数据
 * @note
 * @param
 * @retval
 */
void WT901RecDataFunc(uint8_t *adata, uint8_t size, WT901RecData *rec_imu){
    // 帧头不对
    if(adata[0] != 0x55){
        return;
    }

    const uint8_t total_rec_length = 2;
    // 数据长度不对
    if(size != total_rec_length * SINGLE_REC_LENGTH){
        return;
    }

    // 通信帧数
    rec_imu->frame_counter++;

    memcpy(&stcGyro, &adata[2], 6);
    rec_imu->Gyro.wx = ((float)stcGyro.w[0] / 32768.0 * 2000) * 3.14 / 180.0;
    rec_imu->Gyro.wy = ((float)stcGyro.w[1] / 32768.0 * 2000) * 3.14 / 180.0;
    rec_imu->Gyro.wz = ((float)stcGyro.w[2] / 32768.0 * 2000) * 3.14 / 180.0;

    memcpy(&stcAngle, &adata[13], 6);
    rec_imu->Angle.roll = (float)stcAngle.roll / 32768.0 * 180;
    rec_imu->Angle.pitch = (float)stcAngle.pitch / 32768.0 * 180;
    rec_imu->Angle.yaw = (float)stcAngle.yaw / 32768.0 * 180;
}
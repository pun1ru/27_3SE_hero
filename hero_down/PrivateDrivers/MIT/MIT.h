#ifndef _MIT_H_
#define _MIT_H_

#include "CAN_driver.h"

/* ====== 批量操作封装 ====== */

/**
 * @brief 批量 MIT 力矩控制：向 0x01~0x04 发送纯力矩帧
 * @param hcan  CAN 总线句柄
 * @param torq  4 电机力矩值 [Nm]，下标 0=LF 1=RF 2=RB 3=LB
 * @note  pos/vel/Kp/Kd 置零，仅发送前馈力矩 Tff
 */
void DM_MITControl_JointsSendTorq(FDCAN_HandleTypeDef* hcan, const float torq[4]);

/**
 * @brief 批量电机启动
 * @param hcan  CAN 总线句柄
 * @param ids   电机 CAN ID 数组
 * @param count 数组长度
 */
void Motors_Start(FDCAN_HandleTypeDef* hcan, const uint16_t* ids, uint8_t count);

/**
 * @brief 批量电机锁定
 */
void Motors_Lock(FDCAN_HandleTypeDef* hcan, const uint16_t* ids, uint8_t count);

/**
 * @brief 批量清除电机错误
 */
void Motors_ClearError(FDCAN_HandleTypeDef* hcan, const uint16_t* ids, uint8_t count);

#endif

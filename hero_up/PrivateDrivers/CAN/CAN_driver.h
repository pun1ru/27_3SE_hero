#ifndef _CAN_DRIVER_H_
#define _CAN_DRIVER_H_

#include "fdcan.h"
void can_bsp_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

/**
 * \brief CAN发送监控结构体（用于 watch 窗口在线查看）
 */
typedef struct {
	uint32_t tx_ok;          // 发送成功计数
	uint32_t tx_fail;        // 发送失败计数
	uint16_t last_fail_id;   // 最后一次失败的目标 ID
	uint32_t err_callback;   // ErrorCallback 触发次数
	uint32_t err_reinit;     // 重初始化次数
} CANTxMonitor;

extern CANTxMonitor canMonitor[3]; // [0]=fdcan1  [1]=fdcan2  [2]=fdcan3

/**
 * \brief CAN邮箱发送
 * \param[in] hcan CAN总线控制句柄
 * \param[in] std_id 目标节点标识符 
 * \param[in] aData 数据缓冲数组
 */ 
void CANTransmit_U8(FDCAN_HandleTypeDef *hcan, uint32_t std_id, uint8_t aData[]);

/**
 * \brief CAN邮箱发送，发送数据以uint16_t为单位
 * \param[in] hcan CAN总线控制句柄
 * \param[in] std_id 目标节点标识符 
 * \param[in] uint16_t data1 data2 data3 data4
 */ 
void CANTransmit_I16(FDCAN_HandleTypeDef *hcan, uint32_t std_id, int16_t output1, int16_t output2, int16_t output3, int16_t output4);


/**
 * \brief CAN初始化启动,配置滤波器
 * \param[in] hcan CAN总线控制句柄
 */
 
void CANInitialize(FDCAN_HandleTypeDef *hcan);

/**
 * \brief 使能CAN接收等待中断
 * \param[in] hcan CAN总线控制句柄 
 */
void CANReceiveEnable(FDCAN_HandleTypeDef *hcan);

#endif

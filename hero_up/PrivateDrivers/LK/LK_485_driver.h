#ifndef __LK_DRIVER_RS485_H
#define __LK_DRIVER_RS485_H

#include <stdint.h>

// 用于存储电机所有状态的结构体
typedef struct {
    // 来自状态1 (0x9A)
    int8_t temperature; // 电机温度 (1℃/LSB)
    int16_t voltage;    // 母线电压 (0.01V/LSB)
    int16_t current;    // 母线电流 (0.01A/LSB)
    uint8_t motorState; // 电机状态 (0x00:开启, 0x10:关闭)
    uint8_t errorState; // 错误标志位

    // 来自状态2 (0x9C) 和所有控制命令的回复
    int16_t torque_current; // 转矩电流
    int16_t speed;          // 电机转速 (1dps/LSB)
    uint16_t encoder;       // 编码器位置

    // 其他数据... (可以根据需要添加)
    int64_t multi_turn_angle; // 多圈角度

    uint32_t frame_counter; // 成功接收的帧计数器
} LkMotor_State_t;

// --- 发送函数声明 ---
uint16_t LK_iqControl_485(uint8_t motor_id, uint8_t *tx_buffer, int16_t iq_control);
// ... 其他发送函数
uint16_t LK_stateRead_2_485(uint8_t motor_id, uint8_t *tx_buffer);
uint16_t LK_stateRead_3_485(uint8_t motor_id, uint8_t *tx_buffer);
uint16_t LK_Motor_off_485(uint8_t motor_id, uint8_t *tx_buffer);
uint16_t LK_Motor_on_485(uint8_t motor_id, uint8_t *tx_buffer);
uint16_t LK_Motor_stop_485(uint8_t motor_id, uint8_t *tx_buffer);
// --- 接收函数声明 (新增) ---
int8_t LK_RS485_Packet_Parse_485(const uint8_t *rx_buffer, uint16_t rx_len, LkMotor_State_t *state);

#endif

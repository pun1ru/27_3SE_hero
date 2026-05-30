#include "distance_check.h"
#include <string.h>
#include "tim.h"
#include <stdint.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
#define M_PI  3.14159265358979323846

uint8_t angle_error_flag=0;
uint8_t distance_error_flag=0;

Distance_Check_t distance_check = {0};
const Distance_Check_t* _distance_check = &distance_check;
void Distance_Error_Handler(void) {
    // 错误处理
    while (1) {
    }
}

//计算CS
uint8_t Distance_Check_Calculate_Checksum(const uint8_t *data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return ~sum + 1;
}


//清空distance_check结构体
void Distance_Check_Clear_Data(Distance_Check_t *distance_check_ptr)
{
	memset(distance_check_ptr->Tx_data, 0, sizeof(distance_check_ptr->Tx_data));
  memset(distance_check_ptr->Rx_data, 0, sizeof(distance_check_ptr->Rx_data));
	distance_check_ptr->RX_byte_counter = 0;
	distance_check_ptr->error_counter = 0;
	distance_check_ptr->flag = DISTANCE_CHECK_INIT;
	distance_check_ptr->data_length[0] = 0;
	distance_check_ptr->data_length[1] = 0;
	distance_check_ptr->data_mode = 0;
	distance_check_ptr->distance_check_translate.distance = 0;
	distance_check_ptr->distance_check_translate.status = 0;
}

// 定义发送和接收数据长度
void Distance_Check_Mode_Check(uint8_t distance_check_data_mode, Distance_Check_t *distance_check_ptr) {
    switch (distance_check_data_mode) {
        case ADDRESS_SET:
            distance_check_ptr->data_length[0] = 5;  // FA 04 01 ADDR CS
            distance_check_ptr->data_length[1] = 4;  // FA 04 81 81 或 FA 84 81 02 FF
            break;
        case DISTANCE_MODIFY:
            distance_check_ptr->data_length[0] = 5;  // FA 04 06 符号 0xXX CS
            distance_check_ptr->data_length[1] = 4;  // FA 04 8B 77 或 FA 84 8B 01 F6 或 FA 84 88 01 F9
            break;
        case RANGE_SET:
            distance_check_ptr->data_length[0] = 5;  // FA 04 09 Range CS
            distance_check_ptr->data_length[1] = 4;  // FA 04 89 79 或 FA 84 89 01 F8
            break;
        case FREQUENCY_SET:
            distance_check_ptr->data_length[0] = 5;  // FA 04 0A Freq CS
            distance_check_ptr->data_length[1] = 4;  // FA 04 8A 78 或 FA 84 8A 01 F7
            break;
        case RESOLUTION_SET:
            distance_check_ptr->data_length[0] = 5;  // FA 04 0C Resolution CS
            distance_check_ptr->data_length[1] = 4;  // FA 04 8C 76 或 FA 84 8C 01 F5
            break;
        case POWER_ON_MEASURE:
            distance_check_ptr->data_length[0] = 5;  // FA 04 0D Start CS
            distance_check_ptr->data_length[1] = 4;  // FA 04 8D 75 或 FA 84 8D 01 F4
            break;
        case SINGLE_MEASURE_1MM:
            distance_check_ptr->data_length[0] = 4;  // ADDR 06 02 CS
            distance_check_ptr->data_length[1] = 11; // ADDR 06 82”3X 3X 3X 2E 3X 3X 3X”CS 或 ADDR 06 82”’E’ ’R’ ’R’ ’-’ ’-’ ’3X’ ’3X’ ”CS
            break;
        case SHUTDOWN:
            distance_check_ptr->data_length[0] = 4;  // ADDR 04 02 CS
            distance_check_ptr->data_length[1] = 4;  // ADDR 04 82 CS
            break;
        case SINGLE_MEASURE_0_1MM:
            distance_check_ptr->data_length[0] = 4;  // ADDR 06 02 CS
            distance_check_ptr->data_length[1] = 11; // ADDR 06 82”3X 3X 3X 2E 3X 3X 3X 3X”CS 或 ADDR 06 82”’E’ ’R’ ’R’ ’-’ ’-’ ‘-‘’3X’ ’3X’ ”CS
            break;
        case CONTINUOUS_MEASURE_1MM:
            distance_check_ptr->data_length[0] = 5;  // ADDR 06 03 CS
            distance_check_ptr->data_length[1] = 11; // ADDR 06 83” 3X 3X 3X 2E 3X 3X 3X”CS 或 ADDR 06 83” ’E’ ’R’ ’R’ ’-’ ’-’ ’3X’ ’3X’”CS
            break;
        case CONTINUOUS_MEASURE_0_1MM:
            distance_check_ptr->data_length[0] = 5;  // ADDR 06 03 CS
            distance_check_ptr->data_length[1] = 12; // ADDR 06 83” 3X 3X 3X 2E 3X 3X 3X 3X”CS 或 ADDR 06 83” ’E’ ’R’ ’R’ ’-’ ’-’ ‘-‘’3X’ ’3X’”CS
            break;
        case LASER_CONTROL:
            distance_check_ptr->data_length[0] = 5;  // ADDR 06 05 LASER CS
            distance_check_ptr->data_length[1] = 5;  // ADDR 06 85 01 CS 或 ADDR 06 85 00 CS
            break;
        default:
            distance_check_ptr->data_length[0] = 0;  // 未知模式
            distance_check_ptr->data_length[1] = 0;  // 未知模式
            break;
    }
}

//设置Tx字符串
void Distance_Check_Tx_Set(Distance_Check_t *distance_check_ptr, uint8_t mode, uint8_t value, uint8_t signal)
{
	Distance_Check_Clear_Data(distance_check_ptr);
	distance_check.data_mode = mode;
  Distance_Check_Mode_Check(mode, &distance_check);
	
    switch (distance_check.data_mode) {
        case ADDRESS_SET:
            distance_check_ptr->Tx_data[0] = FA;
            distance_check_ptr->Tx_data[1] = 0x04;
            distance_check_ptr->Tx_data[2] = 0x01;
            distance_check_ptr->Tx_data[3] = ADDR;
            distance_check_ptr->Tx_data[4] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 4); // CS
            break;

        case DISTANCE_MODIFY:
            distance_check_ptr->Tx_data[0] = FA;
            distance_check_ptr->Tx_data[1] = 0x04;
            distance_check_ptr->Tx_data[2] = 0x06;
            distance_check_ptr->Tx_data[3] = signal; // 符号（正或者负，负为0x2d，正为0x2b）
            distance_check_ptr->Tx_data[4] = value; // 修正值，一个字节
            distance_check_ptr->Tx_data[5] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 5); // CS
            break;

        case RANGE_SET:
            distance_check_ptr->Tx_data[0] = FA;
            distance_check_ptr->Tx_data[1] = 0x04;
            distance_check_ptr->Tx_data[2] = 0x09;
            distance_check_ptr->Tx_data[3] = value; // Range: 0x05, 0x10, 0x30, 0x50, 0x80
            distance_check_ptr->Tx_data[4] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 4); // CS
            break;

        case FREQUENCY_SET:
            distance_check_ptr->Tx_data[0] = FA;
            distance_check_ptr->Tx_data[1] = 0x04;
            distance_check_ptr->Tx_data[2] = 0x0A;
            distance_check_ptr->Tx_data[3] = value; // Freq: 0x05, 0x10, 0x20
            distance_check_ptr->Tx_data[4] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 4); // CS
            break;

        case RESOLUTION_SET:
            distance_check_ptr->Tx_data[0] = FA;
            distance_check_ptr->Tx_data[1] = 0x04;
            distance_check_ptr->Tx_data[2] = 0x0C;
            distance_check_ptr->Tx_data[3] = value; // Resolution: 1(1mm), 2(0.1mm)
            distance_check_ptr->Tx_data[4] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 4); // CS
            break;

        case POWER_ON_MEASURE:
            distance_check_ptr->Tx_data[0] = FA;
            distance_check_ptr->Tx_data[1] = 0x04;
            distance_check_ptr->Tx_data[2] = 0x0D;
            distance_check_ptr->Tx_data[3] = value; // Start: 0(关闭), 1(开启)
            distance_check_ptr->Tx_data[4] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 4); // CS
            break;

        case SINGLE_MEASURE_1MM:
            distance_check_ptr->Tx_data[0] = ADDR;
            distance_check_ptr->Tx_data[1] = 0x06;
            distance_check_ptr->Tx_data[2] = 0x02;
            distance_check_ptr->Tx_data[3] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 3); // CS
            break;

        case SHUTDOWN:
            distance_check_ptr->Tx_data[0] = ADDR;
            distance_check_ptr->Tx_data[1] = 0x04;
            distance_check_ptr->Tx_data[2] = 0x02;
            distance_check_ptr->Tx_data[3] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 3); // CS
            break;

        case SINGLE_MEASURE_0_1MM:
            distance_check_ptr->Tx_data[0] = ADDR;
            distance_check_ptr->Tx_data[1] = 0x06;
            distance_check_ptr->Tx_data[2] = 0x02;
            distance_check_ptr->Tx_data[3] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 3); // CS
            break;

        case CONTINUOUS_MEASURE_1MM:
            distance_check_ptr->Tx_data[0] = ADDR;
            distance_check_ptr->Tx_data[1] = 0x06;
            distance_check_ptr->Tx_data[2] = 0x03;
            distance_check_ptr->Tx_data[3] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 3); // CS
            break;

        case CONTINUOUS_MEASURE_0_1MM:
            distance_check_ptr->Tx_data[0] = ADDR;
            distance_check_ptr->Tx_data[1] = 0x06;
            distance_check_ptr->Tx_data[2] = 0x03;
            distance_check_ptr->Tx_data[3] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 3); // CS
            break;

        case LASER_CONTROL:
            distance_check_ptr->Tx_data[0] = ADDR;
            distance_check_ptr->Tx_data[1] = 0x06;
            distance_check_ptr->Tx_data[2] = 0x05;
            distance_check_ptr->Tx_data[3] = signal; // LASER: 00 关闭，01 开启
            distance_check_ptr->Tx_data[4] = Distance_Check_Calculate_Checksum(distance_check_ptr->Tx_data, 4); // CS
            break;

        default:
            break;
    }
}


//uart sand data
void Distance_Check_Uart_Send_Char(uint8_t mode) {
//    uint8_t i,j;
//		uint8_t data;
		distance_check.data_mode = mode;
		Distance_Check_Mode_Check(distance_check.data_mode,&distance_check);

}
void Distance_Check_Translate_Data(Distance_Check_t *distance_check_ptr)
{
    uint8_t *rx_data = distance_check_ptr->Rx_data;
    uint8_t data_mode = distance_check_ptr->data_mode;

    switch (data_mode) {
        case ADDRESS_SET:
            if (rx_data[0] == FA && rx_data[1] == 0x04 && rx_data[2] == 0x81 && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 1; // 操作成功
            } else if (rx_data[0] == FA && rx_data[1] == 0x84 && rx_data[2] == 0x81 && 
                       Distance_Check_Calculate_Checksum(rx_data, 4) == rx_data[4]) {
                distance_check_ptr->distance_check_translate.status = 0; // 写入地址错误返回
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case DISTANCE_MODIFY:
            if (rx_data[0] == FA && rx_data[1] == 0x04 && rx_data[2] == 0x8B && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 1; // 操作成功
            } else if (rx_data[0] == FA && rx_data[1] == 0x84 && rx_data[2] == 0x8B && 
                       Distance_Check_Calculate_Checksum(rx_data, 4) == rx_data[4]) {
                distance_check_ptr->distance_check_translate.status = 0; // 操作失败
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case RANGE_SET:
            if (rx_data[0] == FA && rx_data[1] == 0x04 && rx_data[2] == 0x89 && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 1; // 操作成功
            } else if (rx_data[0] == FA && rx_data[1] == 0x84 && rx_data[2] == 0x89 && 
                       Distance_Check_Calculate_Checksum(rx_data, 4) == rx_data[4]) {
                distance_check_ptr->distance_check_translate.status = 0; // 操作失败
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case FREQUENCY_SET:
            if (rx_data[0] == FA && rx_data[1] == 0x04 && rx_data[2] == 0x8A && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 1; // 操作成功
            } else if (rx_data[0] == FA && rx_data[1] == 0x84 && rx_data[2] == 0x8A && 
                       Distance_Check_Calculate_Checksum(rx_data, 4) == rx_data[4]) {
                distance_check_ptr->distance_check_translate.status = 0; // 操作失败
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case RESOLUTION_SET:
            if (rx_data[0] == FA && rx_data[1] == 0x04 && rx_data[2] == 0x8C && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 1; // 操作成功
            } else if (rx_data[0] == FA && rx_data[1] == 0x84 && rx_data[2] == 0x8C && 
                       Distance_Check_Calculate_Checksum(rx_data, 4) == rx_data[4]) {
                distance_check_ptr->distance_check_translate.status = 0; // 操作失败
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case POWER_ON_MEASURE:
            if (rx_data[0] == FA && rx_data[1] == 0x04 && rx_data[2] == 0x8D && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 1; // 操作成功
            } else if (rx_data[0] == FA && rx_data[1] == 0x84 && rx_data[2] == 0x8D && 
                       Distance_Check_Calculate_Checksum(rx_data, 4) == rx_data[4]) {
                distance_check_ptr->distance_check_translate.status = 0; // 操作失败
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case SINGLE_MEASURE_1MM:
            if (rx_data[0] == ADDR && rx_data[1] == 0x06 && rx_data[2] == 0x82 && 
                Distance_Check_Calculate_Checksum(rx_data, 10) == rx_data[10]) {
                char buffer[9];
                memcpy(buffer, &rx_data[3], 8);
                buffer[8] = '\0';
                if (buffer[0] == 'E' && buffer[1] == 'R' && buffer[2] == 'R') {
                    distance_check_ptr->distance_check_translate.status = 0; // 错误返回
                } else {
                    distance_check_ptr->distance_check_translate.distance = strtod(buffer, NULL);
                    distance_check_ptr->distance_check_translate.status = 1; // 正确返回
                }
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case SHUTDOWN:
            if (rx_data[0] == ADDR && rx_data[1] == 0x04 && rx_data[2] == 0x82 && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 1; // 正确返回
            } else if (rx_data[0] == ADDR && rx_data[1] == 0x04 && rx_data[2] == 0x82 && 
                       Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                distance_check_ptr->distance_check_translate.status = 0; // 错误返回
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case SINGLE_MEASURE_0_1MM:
            if (rx_data[0] == ADDR && rx_data[1] == 0x06 && rx_data[2] == 0x82 && 
                Distance_Check_Calculate_Checksum(rx_data, 10) == rx_data[10]) {
                char buffer[9];
                memcpy(buffer, &rx_data[3], 8);
                buffer[8] = '\0';
                if (buffer[0] == 'E' && buffer[1] == 'R' && buffer[2] == 'R') {
                    distance_check_ptr->distance_check_translate.status = 0; // 错误返回
                } else {
                    distance_check_ptr->distance_check_translate.distance = strtod(buffer, NULL);
                    distance_check_ptr->distance_check_translate.status = 1; // 正确返回
                }
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case CONTINUOUS_MEASURE_1MM:
            if (rx_data[0] == ADDR && rx_data[1] == 0x06 && rx_data[2] == 0x83 && 
                Distance_Check_Calculate_Checksum(rx_data, 10) == rx_data[10]) {
                char buffer[9];
                memcpy(buffer, &rx_data[3], 8);
                buffer[8] = '\0';
                if (buffer[0] == 'E' && buffer[1] == 'R' && buffer[2] == 'R') {
                    distance_check_ptr->distance_check_translate.status = 0; // 错误返回
                } else {
                    distance_check_ptr->distance_check_translate.distance = strtod(buffer, NULL);
                    distance_check_ptr->distance_check_translate.status = 1; // 正确返回
                }
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case CONTINUOUS_MEASURE_0_1MM:
            if (rx_data[0] == ADDR && rx_data[1] == 0x06 && rx_data[2] == 0x83 && 
                Distance_Check_Calculate_Checksum(rx_data, 10) == rx_data[10]) {
                char buffer[9];
                memcpy(buffer, &rx_data[3], 8);
                buffer[8] = '\0';
                if (buffer[0] == 'E' && buffer[1] == 'R' && buffer[2] == 'R') {
                    distance_check_ptr->distance_check_translate.status = 0; // 错误返回
                } else {
                    distance_check_ptr->distance_check_translate.distance = strtod(buffer, NULL);
                    distance_check_ptr->distance_check_translate.status = 1; // 正确返回
                }
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        case LASER_CONTROL:
            if (rx_data[0] == ADDR && rx_data[1] == 0x06 && rx_data[2] == 0x85 && 
                Distance_Check_Calculate_Checksum(rx_data, 3) == rx_data[3]) {
                if (rx_data[3] == 0x01) {
                    distance_check_ptr->distance_check_translate.status = 1; // 正确返回
                } else if (rx_data[3] == 0x00) {
                    distance_check_ptr->distance_check_translate.status = 0; // 错误返回
                } else {
                    distance_check_ptr->distance_check_translate.status = 0; // 其他错误
                }
            } else {
                distance_check_ptr->distance_check_translate.status = 0; // 其他错误
            }
            break;

        default:
            distance_check_ptr->distance_check_translate.status = 0; // 未知模式
            break;
    }
}



void State_Machine(Distance_Check_t *distance_check_ptr,uint8_t mode, uint8_t value, uint8_t signal)
{
	switch(distance_check.flag)
	{
		case DISTANCE_CHECK_INIT:
		{
			break;
		}
		case DISTANCE_START:
		{
//			Distance_Check_Uart_Init();
//			HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // 禁用中断
			Distance_Check_Tx_Set(distance_check_ptr,RANGE_SET,0x30,0);
			distance_check_ptr->flag = DISTANCE_CHECK_TX;
			break;
		}
		case DISTANCE_CHECK_TX:
		{
			Distance_Check_Uart_Send_Char(distance_check_ptr->data_mode);
			distance_check_ptr->flag = DISTANCE_CHECK_RX_BEGIN;
			break;
		}
		case DISTANCE_CHECK_RX_BEGIN:
		{
//			HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0); // 设置优先级
//			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // 使能中断
			distance_check_ptr->flag = DISTANCE_CHECK_RX_DO;
			break;

		}
		case DISTANCE_CHECK_RX_DO:
		{
			distance_check_ptr->error_counter++;
			if(distance_check_ptr->error_counter>2000) {distance_check_ptr->flag = DISTANCE_CHECK_RX_END; distance_check_ptr->error_counter = 0;}
			
			break;
		}
		case DISTANCE_CHECK_RX_END:
		{
//			distance_check_ptr->error_counter = 0;
//			Distance_Check_Translate_Data(&distance_check);
			if(!distance_check_ptr->distance_check_translate.status) distance_check_ptr->flag = DISTANCE_CHECK_TX;
			else distance_check_ptr->flag = DISTANCE_CHECK_INIT;
			/*END*/
			
		}
	}
}

// RK4 计算过程中的安全参数
#define MAX_INTEGRATION_STEPS 10000 // calculateHeightRK4 的最大模拟步数
#define MIN_INITIAL_VELOCITY_SQRT 1e-6f // 速度平方的最小安全值，防止除零或sqrt(负数)
#define MIN_MASS 1e-6f // 质量的最小安全值
#define MAX_ANGLE_ITERATIONS 1000 // findAngleForHeight 的最大迭代次数

// --- 全局变量 (用于调试和状态反馈，尽量减少使用，或使用RTOS对象管理) ---
// 注意：在RTOS环境中，全局变量的共享需要同步机制。
// 这里为了演示方便，暂时保留，但请考虑使用信号量/互斥锁保护。
float g_aa; // 计数器
float g_error_angle; // 角度误差
float g_calculated_height; // 实际计算出的高度

// 外部声明（假设这些在其他地方定义，用于控制流程）
int com_complet = 1; // 完成标志（原由 super_see_task 管理）
extern uint8_t distance_error_flag; // 假设这是距离错误标志
extern uint8_t outofrange_flag; // 假设这是超距标志
extern uint8_t outofheight_flag; // 假设这是超高标志

// 定义一个结构体来存储弹丸的状态
typedef struct {
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
} Projectile;
/*3.21新更正*/
float airDensity(float z) {
    const float rho0 = 1.225; // kg/m^3, 海平面密度
    const float h = 0.000128; // 密度递减因子, 简化模型
    return rho0 * exp(-h * z);
}

// RK4函数
void rungeKutta(float* x, float* z, float* vx, float* vz, float m, float r, float C, float dt) {
    float k1x, k1z, k1vx, k1vz;
    float k2x, k2z, k2vx, k2vz;
    float k3x, k3z, k3vx, k3vz;
    float k4x, k4z, k4vx, k4vz;

    float ax, az;
    float v = sqrt((*vx) * (*vx) + (*vz) * (*vz));
    float A = M_PI * r * r;
    float rho = airDensity(*z);
    float drag_x = -0.5 * rho * C * A * v * (*vx);
    float drag_z = -0.5 * rho * C * A * v * (*vz);

    ax = drag_x / m;
    az = -9.8 + drag_z / m;

    k1x = *vx;
    k1z = *vz;
    k1vx = ax;
    k1vz = az;

    v = sqrt(((*vx) + 0.5 * dt * k1vx) * ((*vx) + 0.5 * dt * k1vx) + ((*vz) + 0.5 * dt * k1vz) * ((*vz) + 0.5 * dt * k1vz));
    rho = airDensity(*z + 0.5 * dt * k1z);
    drag_x = -0.5 * rho * C * A * v * ((*vx) + 0.5 * dt * k1vx);
    drag_z = -0.5 * rho * C * A * v * ((*vz) + 0.5 * dt * k1vz);

    ax = drag_x / m;
    az = -9.8 + drag_z / m;

    k2x = (*vx) + 0.5 * dt * k1vx;
    k2z = (*vz) + 0.5 * dt * k1vz;
    k2vx = ax;
    k2vz = az;

    v = sqrt(((*vx) + 0.5 * dt * k2vx) * ((*vx) + 0.5 * dt * k2vx) + ((*vz) + 0.5 * dt * k2vz) * ((*vz) + 0.5 * dt * k2vz));
    rho = airDensity((*z) + 0.5 * dt * k2z);
    drag_x = -0.5 * rho * C * A * v * ((*vx) + 0.5 * dt * k2vx);
    drag_z = -0.5 * rho * C * A * v * ((*vz) + 0.5 * dt * k2vz);

    ax = drag_x / m;
    az = -9.8 + drag_z / m;

    k3x = (*vx) + 0.5 * dt * k2vx;
    k3z = (*vz) + 0.5 * dt * k2vz;
    k3vx = ax;
    k3vz = az;

    v = sqrt(((*vx) + dt * k3vx) * ((*vx) + dt * k3vx) + ((*vz) + dt * k3vz) * ((*vz) + dt * k3vz));
    rho = airDensity((*z) + dt * k3z);
    drag_x = -0.5 * rho * C * A * v * ((*vx) + dt * k3vx);
    drag_z = -0.5 * rho * C * A * v * ((*vz) + dt * k3vz);

    ax = drag_x / m;
    az = -9.8 + drag_z / m;

    k4x = (*vx) + dt * k3vx;
    k4z = (*vz) + dt * k3vz;
    k4vx = ax;
    k4vz = az;

    *x += (dt / 6.0) * (k1x + 2 * k2x + 2 * k3x + k4x);
    *z += (dt / 6.0) * (k1z + 2 * k2z + 2 * k3z + k4z);
    *vx += (dt / 6.0) * (k1vx + 2 * k2vx + 2 * k3vx + k4vx);
    *vz += (dt / 6.0) * (k1vz + 2 * k2vz + 2 * k3vz + k4vz);

}


float calculateHeightRK4(float x_target, float V0, float theta, float m, float r, float C, float dt) {
    float x = 0, z = 0;
    float vx = 0;
    vx=V0 * cos(theta);// x方向的速度
    float vz = 0;
    vz=V0 * sin(theta); // z方向的速度
    while (x < x_target && z>-100) {//这里要改，要那个负数
				
        rungeKutta(&x, &z, &vx, &vz, m, r, C, dt);//死在这里面
    }
    return z;
}
float aa;
float error;
float height;
uint32_t heightcount=0;
// 第二个二分法：基于第一个二分法找到的角度，查找在指定射程下打到指定高度需要的角度
float findAngleForHeight(float V0, float targetRange, float targetHeight, float m, float r, float C, float dt) {
		com_complet=0;
		/*在此处增加对错误数值的检查，如果有错误值，填充*/
		if(V0<0 || m<=0 || C<0 || r<=0 ||targetRange<0){
			com_complet=1;
			return -2;//错误码
		}
	
	
    
		float left = -20;
    float right = 90;
    float tolerance = 0.0001;
    
    float mid;
    error =90 ;
	
		distance_error_flag=0;
		angle_error_flag=0;

    while (error > tolerance) {
			aa++;
			heightcount++;
					mid = (left + right) / 2;
					// 将角度转换为弧度
					float theta_rad = 0;
					theta_rad = mid * M_PI / 180;
					height = calculateHeightRK4(targetRange, V0, theta_rad, m, r, C, dt);				
					if (height < targetHeight) {
							left = mid;
					}
					else {
							right = mid;
					}
					if(height<-50){
						com_complet=1;
						return -3;//错误码
					}
			error=right-left;
					//error = sqrt((height - targetHeight)*(height - targetHeight));这么写大错特错，会有奇怪的精度问题，导致无限循环
					/*超时退出机制*/
			if(heightcount>1000)
			{
				heightcount=0;
				com_complet=1;
				return mid;
				}
		}
		com_complet=1;
    return (float)mid;
		
}

// 简单抛物线模型验算函数
uint8_t outofrange_flag=0;
uint8_t outofheight_flag=0;
uint8_t simpleParabolicCheck(float V0, float targetRange, float targetHeight) {
    const float theta = 45 * M_PI / 180; // 45°发射角度
    const float g = 9.8;

    // 计算最大射程
    float maxRange = (V0 * V0 * sin(2 * theta)) / g;
    // 计算最大高度
    float maxHeight = (V0 * V0 * sin(theta) * sin(theta)) / (2 * g);

    if (targetRange > maxRange) {
      //  std::cout << "距离过远无法命中" << std::endl;
			outofrange_flag=1;
        return 0;
    }
    if (targetHeight > maxHeight) {
       // std::cout << "高度过高无法命中" << std::endl;
			outofheight_flag=1;
        return 0;
    }
    return 1;
}

//以下是教程
//int main() {
//    float m = 0.0445;
//    float r = (42.5 / 2) / 1000;
//    float C = 0.43;

//    float V0;d
//    printf("请输入弹丸初速度（单位：m/s）：");
//    if (scanf("%lf", &V0) != 1) {
//        fprintf(stderr, "输入错误，请输入有效的速度。\n");
//        return 1;
//    }

//    float targetRange;
//    printf("请输入目标射程（单位：米）：");
//    if (scanf("%lf", &targetRange) != 1) {
//        fprintf(stderr, "输入错误，请输入有效的距离。\n");
//        return 1;
//    }

//    float theta = findTheta(V0, targetRange, m, r, C);
//    printf("为达到目标射程，发射仰角应为：%f 度。\n", theta * 180 / M_PI);

//    return 0;
//}

/*仿造教程写一个顶层调用函数*/

/*USER INCLUDE BEGIN*/

#include "stm32h7xx_hal.h"
#include "math.h"
#include <stdlib.h>
#include "arm_math.h"

/*USER INCLUDE END*/



/*USER DEFINE BEGIN*/

/*基础*/
#define LASER_UART_LENGTH 12
//#define UART_BAUD_RATE 9600
//#define UART_TX_PIN GPIO_PIN_15
//#define UART_RX_PIN GPIO_PIN_12
//#define UART_GPIO_PORT GPIOB

#define DISTANCE_CHECK_TX_MAX_LENGTH 7
#define DISTANCE_CHECK_RX_MAX_LENGTH 15

/*UART处理数据模式*/
// 设置地址
#define ADDRESS_SET 0x01

// 距离修改
#define DISTANCE_MODIFY 0x02

// 设定量程
#define RANGE_SET 0x03

// 设定频率
#define FREQUENCY_SET 0x04

// 设定分辨率
#define RESOLUTION_SET 0x05

// 设定上电即测
#define POWER_ON_MEASURE 0x06

// 单次测量（1mm）
#define SINGLE_MEASURE_1MM 0x07

// 关机
#define SHUTDOWN 0x08

// 单次测量(0.1mm)
#define SINGLE_MEASURE_0_1MM 0x09

// 连续测量（1mm）
#define CONTINUOUS_MEASURE_1MM 0x0A

//连续测量（0.1mm）
#define CONTINUOUS_MEASURE_0_1MM 0x0B

//控制激光打开或关闭
#define LASER_CONTROL 0x0C

#define FA 0xfa
#define ADDR 0x80

/*状态flag*/
#define DISTANCE_CHECK_INIT 0x00
#define DISTANCE_START 0x01
#define DISTANCE_CHECK_TX 0x02
#define DISTANCE_CHECK_RX_BEGIN 0x03
#define DISTANCE_CHECK_RX_DO 0x04
#define DISTANCE_CHECK_RX_END 0x05

/*USER DEFINE END*/

		
/*USER STRUCT DEFINE*/

#pragma pack(1)
typedef struct
{
	float distance;
	float distance_select;
	uint8_t status;
	
	float angle;
}Distance_Check_Translate_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
	uint8_t Tx_data[DISTANCE_CHECK_TX_MAX_LENGTH];
	uint8_t Rx_data[DISTANCE_CHECK_RX_MAX_LENGTH];
	
	uint8_t data_mode;
	uint8_t data_length[2];//data_length[0] Tx_data_length;
												 //data_length[1] Rx_data_length
	uint8_t RX_byte_counter;
	uint8_t flag;
	uint16_t error_counter;
	
	uint8_t delay;//用来延时
	Distance_Check_Translate_t distance_check_translate;
}Distance_Check_t;
#pragma pack()

/*USER VAL BEGIN*/

extern Distance_Check_t distance_check;

/*USER VAL END*/

/*USER FUNCTION HANDLE BEGIN*/
void Distance_Error_Handler();// 错误处理
uint8_t Distance_Check_Calculate_Checksum(const uint8_t *, size_t);//计算CS

void Distance_Check_Clear_Data(Distance_Check_t *);//清空distance_check结构体
void Distance_Check_Mode_Check(uint8_t, Distance_Check_t *);// 定义发送和接收数据长度
void Distance_Check_Tx_Set(Distance_Check_t *, uint8_t, uint8_t, uint8_t);//设置Tx字符串
void Distance_Check_Uart_Send_Char(uint8_t);//uart sand data

void Distance_Check_Translate_Data(Distance_Check_t *);//翻译接收的数据
float solveLaunchAngle(float sita, float distance);
float calculateLaunchAngle(float V0, float target_distance);//无敌计算
float findAngleForHeight(float V0, float targetRange, float targetHeight, float m, float r, float C, float dt);
uint8_t simpleParabolicCheck(float V0, float targetRange, float targetHeight);
void State_Machine(Distance_Check_t *distance_check_ptr,uint8_t mode, uint8_t value, uint8_t signal);
/*USER FUNCTION HANDLE END*/
#include "tim.h"
#include "usart.h"
 
#include "general_task_include.h"
#include "bsp_dwt.h"

#include "ws2812.h"
// 定义一个枚举类型来表示不同的音乐/声音ID

typedef enum {
    SOUND_NONE = 0,
    SOUND_ERROR_TYPE_A,      // 机构A的错误声音
    SOUND_ERROR_TYPE_B,      // 机构B的错误声音
    SOUND_STARTUP,           // 开机提示音
    SOUND_SUCCESS            // 操作成功提示音
} MusicID_t;

// 声明一个全局的队列句柄
extern QueueHandle_t g_musicQueue;
#ifndef __CIRCUIT_MONITOR_H__
#define __CIRCUIT_MONITOR_H__
typedef struct
{ 
	struct
	{
		uint16_t chassisMotor_frame_counter[4];
		uint16_t yawMotor_frame_counter;
		uint16_t stirMotor_frame_counter;
		uint16_t fricMotor_frame_counter[3];
		uint16_t pitchMotor_frame_counter;
		uint16_t superCapacity_frame_counter;
		uint16_t smallpitchMotor_frame_counter;
	}CircuitCounterPtr;
	struct
	{
		uint8_t chassisMotorError[4];
		uint8_t yawMotorError;
		uint8_t stirMotorError;
		uint8_t fricMotorError[3];
		uint8_t pitchMotorError;
		uint8_t superCapacityError;
		uint8_t smallpitchMotorError;
	}ifCircuitError;
}CircuitMonitor;
#endif // __CIRCUIT_MONITOR_H__
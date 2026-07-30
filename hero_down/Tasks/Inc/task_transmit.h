#ifndef TASK_TRANSMIT_H_
#define TASK_TRANSMIT_H_

#include <stdint.h>

/**
 * @brief   调试 UART 帧格式选择
 * @note    在 task_transmit.c 文件开头最多启用一个 DEBUG_FRAME_* 宏。
 * @note    DEBUG_FRAME_UP 为控制调试帧，帧头为 0xAABB，长度 38 字节。
 * @note    DEBUG_FRAME_SYSID 为系统辨识调试帧，帧头为 0xAABB，长度 30 字节。
 */

/**
 * @brief 电机的CAN信号帧发送
 * @note  总线挂载情况：CAN1-(4*M3508底盘电机)+(GM6020yaw电机+电容控制板通信)；CAN2-(GM6020pitch电机+2*M3508摩擦轮电机)+MS4010拨盘电机
 */
void MotorControlCANSend(void);

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define UPPER_PC_COMM_SEND_LENGTH 24+4
#define UPPER_PC_COMM_REC_LENGTH 20
#define UPPER_PC_COMM_REC_SOF 0x33
#define UPPER_PC_COMM_REC_EOF 0xEE
/*all receive control frame type*/
#define NO_AIM_NOR_SHOOT 0x00
#define AIM_BUT_NOT_SHOOT 0x01
#define AIM_AND_SHOOT 0x02
/*all send task mode frame type*/
#define AUTO_AIM 0x01
#define BUFF_AIM 0x02
#define HANG_AIM 0x03

#define RED_TEAM_FRAME 0xAA
#define BLUE_TEAM_FRAME 0xBB
/**
 * @brief 上位机通信结构体
 */
typedef struct
{
	uint16_t rec_counter;
	struct
	{
		uint8_t sof;
		uint8_t reboot;
		uint8_t reserved1[2];
		float target_pitch_angle_d; 
		float target_yaw_angle_d;
		float yaw_speed_error;
		uint8_t shoot_mode;
		uint8_t reserved2;
		uint8_t aiming_state;
		uint8_t eof;
	}Receive;
	struct
	{
        uint8_t sof;//起始帧,0x33
		uint8_t task_mode;//任务模式,暂定0是普通模式,2是吊射模式
		uint8_t self_team;//队伍的颜色,0xAA红队，0xBB蓝队
		uint8_t _reserved;//保留位
		float bullet_speed;//弹速
		float gimbal_roll_d;//roll角
		float gimbal_yaw_d;//yaw角
		float gimbal_pitch_d;//pitch角	
		float gimbal_yaw_dps;//yaw角速度
		uint8_t reserved[3];//保留位
        uint8_t eof;//结束帧,0xEE	
		
	}Send;
}UpperComputerComm;

extern const UpperComputerComm* _upperComputerComm;

void DebugTask(void* argument);
void UIOperationTask(void* argument);

#endif

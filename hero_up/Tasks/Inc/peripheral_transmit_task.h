#ifndef _PERIPHERAL_TRANSMIT_TASK_
#define _PERIPHERAL_TRANSMIT_TASK_
/**
 * @brief 电机的CAN信号帧发送
 * @note  总线挂载情况：CAN1-(4*M3508底盘电机)+(GM6020yaw电机+电容控制板通信)；CAN2-(GM6020pitch电机+2*M3508摩擦轮电机)+MS4010拨盘电机
 */
void MotorControlCANSend(void);
/**
 * @brief LED闪烁实现
 */
void LEDShow(void);

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
#define MCU_FRAME_LEN (1U+21U)  /* 与底盘板同步, 去掉未使用的30B */
//遥控器种类帧+遥控器帧+裁判系统帧(底盘用)=双板通信帧
/**
 * @brief 上位机通信结构体
 */
typedef struct
{
	uint16_t rec_counter;
	struct
	{
//		uint8_t sof;
//		uint8_t reserved1[3];
//		float target_pitch_angle_d; 
//		float target_yaw_angle_d;
//		uint8_t shoot_mode;
//		uint8_t shot_out_post;		//shot_bufff_mode
//		uint8_t aiming_state;		//aiming_state 0x11有通讯没相机	0x22自瞄正常无目标	0x33锁定目标
//		uint8_t eof;
		
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
		uint8_t cam_target;//摄影目标端  1=上 2=中 3=下
		float bullet_speed;//弹速
		float gimbal_roll_d;//roll角
		float gimbal_yaw_d;//yaw角
		float gimbal_pitch_d;//pitch角	
		float gimbal_yaw_dps;//yaw角速度
		uint8_t reserved[3];//保留位
        uint8_t eof;//结束帧,0xEE	
		
	}Send;
}UpperComputerComm;
#endif

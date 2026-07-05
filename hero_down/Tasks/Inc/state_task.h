#ifndef _STATE_TASK_H_
#define _STATE_TASK_H_

#include "stdint.h"
/*---------------------------------------------------------------------------monitor task-----------------------------------------------------------------------------------*/
/**
 * @brief 任务监视器结构体，能读取每个任务的计数器，任务周期及根据条件判断任务是否卡死给出标志位
 */
typedef struct
{ 
	struct
	{
		const uint16_t* _remote_rec_task;
		const uint16_t* _state_task;
		const uint16_t* _decision_task;
		const uint16_t* _control_task;
		const uint16_t* _estimate_task;
		const uint16_t* _imu_task;
		const uint16_t* _debug_task;
		const uint16_t* _upper_pc_comm_task;
		const uint16_t* _ui_operation_task;
		const uint16_t* _music_task;
	}TaskFrameCounterPtr;
	struct
	{
		const uint32_t* _remote_rec_task;
		const uint32_t* _state_task;
		const uint32_t* _decision_task;
		const uint32_t* _control_task;
		const uint32_t* _estimate_task;
		const uint32_t* _imu_task;
		const uint32_t* _debug_task;
		const uint32_t* _upper_pc_comm_task;
		const uint32_t* _ui_operation_task;		
		const uint32_t* _music_task;
	}TaskRunPeriodPtr;
}TaskMonitor;
 
/*任务是否卡死标志位或周期扰乱，判断条件根据每个任务的实际监测需求自己写，位掩码如下*/
#define REMOTE_REC_TASK_MASK 0x01
#define STATE_TASK_MASK 	 0x02
#define DECISION_TASK_MASK	 0x04
#define CONTROL_TASK_MASK 	 0x08
#define ESTIMATE_TASK_MASK 	 0x100
#define IMU_TASK_MASK		 0x10
#define DEBUG_TASK_MASK		 0x20
#define UPPER_COMM_TASK_MASK 0x40
#define UI_OPERATION_TASK_MASK 0x80
#define MUSIC_TASK_MASK 0x90
/*---------------------------------------------------------------------------state task-----------------------------------------------------------------------------------*/
/*define different state*/
/*control terminal*/
#define CONTROL_STOP		0
#define CONTROL_FROM_REMOTE	1
#define CONTROL_FROM_PC		2
/*chassis mode*/
#define CHASSIS_FOLLOW		0
#define CHASSIS_FOLLOW_BACK 1  //反向跟随，机体背向云台朝向
#define CHASSIS_REVOLVE 	2
#define CHASSIS_SEPARATE	3

/*pitch mode*/
#define PITCH_SEPARATE 0
#define PITCH_LINK 1

/*发射相关*/
#define FRIC_OFF 0
#define FRIC_ON 1
#define STIR_LOCK 0
#define STIR_ANGLE_CONTROL 1
#define STIR_REVERSE 2
#define STIR_MAD_DOG 3
//3.20
#define FOLLOW_ON 1
#define FOLLOW_OFF 0 //这是平行的意思
#define SNIPER_ON 1//需要的话在这里那个开局保护
#define SNIPER_OFF 0
#define LENS_ON 1
#define LENS_OFF 0
//5.17 C键算法基地自瞄
#define NEW_AUTO_AIMING_OFF 0
#define NEW_AUTO_AIMING_ON  1

//5.28
#define BULLET_ADAPTION_ON 1
#define BULLET_ADAPTION_OFF 0
/*其他状态*/
#define NORMAL_MODE 0
#define OUTPOSE_MODE 1
#define CAPACITY 0
#define NO_CAPACITY 1
#define ROBOT_JOINT_MODE_NORMAL 0
#define ROBOT_JOINT_MODE_PRECLIMB 1
#define ROBOT_JOINT_MODE_CLIMB 2
#define ROBOT_JOINT_MODE_OUTCLIMB 3
#define ROBOT_STAND_MODE_NORMAL    0
#define ROBOT_STAND_MODE_PRE_STAIR 1
#define ROBOT_STAND_MODE_STAIR_UP  2
#define ROBOT_STAND_MODE_PRE_DOWN_STAIR 3
#define ROBOT_JUMP_MODE_OFF 0
#define ROBOT_JUMP_MODE_ON 1
#define MOUSE_FIX_OFF 0
#define MOUSE_FIX_ON  1
#define WORLD_ENABLE_OFF 0
#define WORLD_ENABLE_ON  1
#define CAM_TARGET_UP   1
#define CAM_TARGET_MID  2
#define CAM_TARGET_DOWN 3
#pragma pack(1)
/**
 * @brief 状态机结构体，标识全车变量
 */
typedef struct
{
	unsigned ctrl_terminal : 2;		//当前控制机器人行为终端：停止控制（保护），遥控器控制，电脑控制
	unsigned chassis_mode : 2;		//底盘当前模式
	unsigned fric_mode : 1;
	unsigned stir_mode : 2;
	unsigned auto_slope : 1;
	unsigned aim_mode : 1;
	unsigned capacity_mode : 1;
	unsigned cam_target : 2;		//摄影目标：上/中/下
	unsigned world_enable : 1;	//ch4>0.1 使能世界坐标系yaw目标（来自上板）
	unsigned reserved : 2;
	unsigned bullet_adaption_mode:1;
	
	unsigned follow:1;
	unsigned sniper:1;
	unsigned lens:1;
	unsigned joint_mode:2;
	unsigned stand_mode:2;
	unsigned jump_mode:1;
	unsigned mouse_fix:1;		//sniper鼠标锁定
}RobotState;
#pragma pack()
typedef struct
{int xv_ni_heart;
	int count;
}xv_ni;
#endif

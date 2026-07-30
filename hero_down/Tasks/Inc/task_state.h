#ifndef TASK_STATE_H_
#define TASK_STATE_H_

#include <stdint.h>
/*---------------------------------------------------------------------------state task-----------------------------------------------------------------------------------*/
//新qm状态机宏定义,chassis,ctrlterminal已生成

#define RC_LOST 1
#define RC_OK 0

#define CAN_ENABLE 1
#define CAN_DISABLE 0
//关节
#define JOINT_NORMAL 0
#define JOINT_PRESTAIR 1
#define JOINT_STAIRUP 2 

#define STAND_NORMAL 0
#define STAND_HIGH 1

//发射
#define FRIC_OFF 0
#define FRIC_ON 1
#define STIR_LOCK 0
#define STIR_ANGLE_CONTROL 1
#define STIR_REVERSE 2

//瞄准
//吊射
#define SNIPER_ON 1
#define SNIPER_OFF 0
#define MOUSE_FIX_OFF 0
#define MOUSE_FIX_ON  1
#define WORLD_ENABLE_OFF 0
#define WORLD_ENABLE_ON  1
#define CAM_TARGET_UP   1
#define CAM_TARGET_MID  2
#define CAM_TARGET_DOWN 3
#define AUTO_AIM_OFF 0
#define AUTO_AIM_ON  1

//电容
#define CAPACITY 0
#define NO_CAPACITY 1


//老宏定义
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

//3.20
#define FOLLOW_ON 1
#define FOLLOW_OFF 0 //这是平行的意思
#define LENS_ON 1
#define LENS_OFF 0
/*其他状态*/
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
	unsigned : 1;
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

extern const RobotState* const _robotState;
extern int crawler_rotate_flag;

typedef struct
{int xv_ni_heart;
	int count;
}xv_ni;

void StateMachineTask(void* argument);

#endif

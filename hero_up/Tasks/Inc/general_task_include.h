#ifndef _GENERALTASKINCLUDE_H_
#define _GENERALTASKINCLUDE_H_

/*该头文件只inlcude各任务共需的头文件*/
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

#include "task.h"
#include "event_groups.h"

#include "general_define.h"
#include "general_config_label.h"
 
#include "state_task.h"
#include "peripheral_transmit_task.h"
#include "peripheral_receive_task.h"
#include "music_task.h"
#include "judge_receive.h"
#include "task_estimate.h"
#include "task_decision.h"
#include "task_control.h"
#include "distance_check.h"

/* 算法/控制模块 */
#include "algorism.h"
#include "MadWick.h"
#include "pid.h"
#include "shoot_speed_best_contrl.h"
#include "adrc.h"
#include "arm_math.h"

/* MainControl + 驱动 */
#include "robot_control_task.h"
#include "DMJ4310.h"
#include "CAN_driver.h"

/*该头文件只extern共享各任务共需的全局变量*/
extern TaskHandle_t decisionTaskHandle;
extern TaskHandle_t estimateTaskHandle;
extern TaskHandle_t stateMachineTaskHandle;
extern TaskHandle_t controlTaskHandle;
extern TaskHandle_t upperPCCommTaskHandle;
extern TaskHandle_t remoteRecTaskHandle;
extern TaskHandle_t uiOperationTaskHandle;
extern TaskHandle_t monitorTaskHandle;
extern TaskHandle_t imuTaskHandle;

void StateMachineTask(void* argument);
void DecisionTask(void* argument);
void ControlTask(void* argument);
void UIOperationTask(void* argument);
void UpperPCCommTask(void* argument);
void RemoteRecTask(void* argument);
void MonitorTask(void* argument);
void IMUTask(void* argument);
void DebugTask(void* argument);
void MusicTask(void* argument);

/*每个task.c文件中定义的全局变量只能在当前源文件中被操作更改，其它task只能通过一下全局指针常量读取,各const ptr的初始化在对应的.c文件中*/
extern TaskMonitor* _taskMonitor;		//该结构体内部均为常量指针
extern const DJIGMotorRec *_chassisMotorRec;
extern const RobotState* _robotState;
extern const DMJ4310MotorRec* _stirMotorRec;
extern const DJIGMotorRec *_chassisMotorRec;
extern const DJIGMotorRec *_fricMotorRec;
extern const DJIGMotorRec* _pitchMotorRec;
extern const DJIGMotorRec* _smallpitchMotorRec;
extern const NormRemoteCmd* _normRemoteCmd;
extern const ChassisControl* _chassisControl;
extern const GimbalControl* _gimbalControl;
extern const ShootControl* _shootControl;
extern const Pose* _gimbalPose;
extern const UpperComputerComm* _upperComputerComm;
extern const SuperCapacity* _superCapacity;
extern const DataFromJudge* _bulletSpeed;
extern const Distance_Check_t* _distance_check;
extern const RobotState* _lastRobotState;
extern RobotState lastRobotState;
extern EventGroupHandle_t remoteRecEventGroup;
extern const WorldGimbal* _worldGimbal;

#endif

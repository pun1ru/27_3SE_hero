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
#include "device_define.h"
#include "robot_define.h"

#include "initial_task.h"
#include "task_state.h"
#include "task_transmit.h"
#include "task_receive.h"
#include "judge_receive.h"
#include "task_monitor.h"
#include "task_estimate.h"
#include "task_decision.h"
#include "task_control.h"
#include "distance_check.h"
#include "UI_design.h"

//包含共通的算法文件
#include "algorism.h"
#include "pid.h"
#include "adrc.h"
#include "arm_math.h"
#include "gimbalControl.h"
#include "jointControl.h"
#include "DM_driver.h"
#include "MIT.h"
#include "CAN_driver.h"
#include "chassisControl.h"
#include "stirControl.h"

/* QP 状态机框架 */
#include "qpc_init.h"      /* QpInit(), QF_onStartup 等 */
#include "decision_ao.h"   /* DecisionAO 类型, 信号枚举, AO_DecisionAO */

void StateMachineTask(void* argument);
void DecisionTask(void* argument);
void ControlTask(void* argument);
void UIOperationTask(void* argument);
void UpperPCCommTask(void* argument);
void RemoteRecTask(void* argument);
void MonitorTask(void* argument);
void IMUTask(void* argument);
void DebugTask(void* argument);

/*每个task.c文件中定义的全局变量只能在当前源文件中被操作更改，其它task只能通过一下全局指针常量读取,各const ptr的初始化在对应的.c文件中*/
extern const DJIGMotorRec* _pitchMotorRec;
extern const DJIGMotorRec* _smallpitchMotorRec;
extern const JointControl* _jointControl;
extern const DataFromJudge* _bulletSpeed;
extern const Distance_Check_t* _distance_check;
extern EventGroupHandle_t remoteRecEventGroup;

#endif

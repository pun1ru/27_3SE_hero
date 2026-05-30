#include "robot_control_task.h"
#include "adrc.h"
#include "algorism.h"
#include "pid.h"
#include "state_task.h"
#include "stdio.h"
#include "math.h"
#include "tim.h"
#include "general_task_include.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"
#include <arm_math.h>
#include <stdint.h>
#include "CAN_driver.h"
#include "DMJ4310.h"
#include "shoot_speed_best_contrl.h"
#include "LK_driver.h"
#include "rotation_Martix.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"
#include "jointControl.h"
extern UpperComputerComm upperComputerComm;
extern DMJ4310MotorRec DMyawMotorRec;
extern const DMJ4310MotorRec* _DMyawMotorRec;
extern const DMJ4310MotorRec* _jointMotorEec;
extern volatile float gimbal_yaw_target_rx_d;  /* RS485接收的目标yaw角度 */

/*---------------------------------------------------------------------------decision task-----------------------------------------------------------------------------------*/
ChassisControl chassisControl={0};
const ChassisControl* _chassisControl = &chassisControl;
GimbalControl gimbalControl={0};
const GimbalControl* _gimbalControl = &gimbalControl;
extern DJIGMotorRec yawMotorRec;
extern const DJIGMotorRec* _yawMotorRec;
extern uint8_t angle_error_flag;
extern uint8_t distance_error_flag;
extern RobotState robotState;
extern void BulletSpeedReceive(void);
extern Pose gimbalPose;
ShootControl shootControl={0};            //hxg
const ShootControl* _shootControl = &shootControl;
JointControl jointControl={0};
const JointControl* _jointControl = &jointControl;
SmoothFilter MouseFilterX={0};
SmoothFilter MouseFilterY={0};

AverageFilter PowerFilter={0};
float pitch_angle_from_match=0;
static uint8_t c_key_aim_yaw_lock = 0;  /* C键自瞄yaw保护锁: 1=禁止ALLHighFreqCal覆盖yaw目标 */
leastSquareLinear bulletSpeedAdaptation = {
.x = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30},
.count=0
};

static void DecisionInit(void);

static void ChassisInputUpdate(void);
static void JointInputUpdate(void);
static void GimbalInputUpdate(void);
static void ShootInputUpdate(void);

JointBodyTarget g_joint_body_target_cmd = {0};
static JointBodyState g_joint_body_state_obs = {0};
float g_joint_body_pitch_ctrl_d = 0.0f;

static void StirTargetAngleSet(void);
static void GetStirRealAngle(void);

/**
* @brief 机器人目标决策设定，给定各输入值
 */
void DecisionTask(void* argument)
{
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count;	
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._decision_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._decision_task = &this_tick_count;

	DecisionInit();
	BulletKF_Init();//初始化卡尔曼滤波滤波滤波
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	
	while(1)
	{
		/*主任务进程*/
		ChassisInputUpdate();		
		JointInputUpdate();
		GimbalInputUpdate();
		ShootInputUpdate();

		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
		vTaskDelayUntil(&current_tick_count, DECISION_TASK_PERIOD_SET);
	}
}

 
/**
 * @brief 决策任务相关初始化
 */
static void DecisionInit(void)
{
	PIDInitialize(&chassisControl.ChassisFollowControl.follow_speed_need_pid, -0.02, 0, 0.008, 0, 3); /* RS485 IMU已确认正常 */
	PIDInitialize(&chassisControl.ChassisFollowControl.follow_speed_need_pid, -0.02, 0, 0.008, 0, 3); /* RS485 IMU已确认正常 */
	PIDInitialize(&chassisControl.GimbalCoordinateInput.speed_x_compensate_pid, 0.1, 0.1, 0, 1, 3);
	PIDInitialize(&chassisControl.GimbalCoordinateInput.speed_y_compensate_pid, 0.1, 0.1, 0, 1, 3);
	SmoothFilterInitialize(&MouseFilterX,0.7);
	SmoothFilterInitialize(&MouseFilterY,0.7);
}

/**
 * @brief 底盘相关输入决策更新
 * @note:  默认电机顺序: 1---2	  ^
 *				         |	 |    |
 *                       |   |    |
 *                       4---3    |
 * 			1 4号电机向前旋转输出为正，2 3向后旋转输出为负（根据实际情况更新，影响正逆运动学解算）
 * 线速度x,y单位m/s 自旋速度w单位rps
 *					^ y
 *					|
 *					|--->x
 */
static void ChassisInputUpdate(void)
{
	/* RS485 IMU yaw 覆盖已移至 GimbalPoseUpdate()，此处不再重复 */

	/*遥操作输入->云台坐标系下的输入->底盘坐标系下的输入->四个底盘电机的输入*/
	
	/*----------------------------------------------------------------------move---------------------------------------------------------------------------------------*/
	/*云台坐标系下的输入*/
	float delta_angle_cos, delta_angle_sin;
	arm_sin_cos_f32(-chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d, &delta_angle_sin, &delta_angle_cos);	
	
	uint8_t is_input_flag = 1,is_input_flag_x = 1, is_input_flag_y = 1;
	
	//3508电机极限转速大约在8500左右，这样计算出合成速度向量的最大值为8500/60/14*2*3.14*0.076=2.5
	float max_linear_speed_mps_y = 3;
	float max_linear_speed_mps_x = 3;
	
	//当设定方向速度和当前底盘速度作差，只有小于该设定值时才能继续累加目标速度
	float max_speed_x_rps_error = 0.04 / (0.7 * fabs(chassisControl.ChassisEstimate.speed_x_mps) + 0.5) + 0.15;
	//float max_speed_y_rps_error = 0.06 / (0.7 * fabs(chassisControl.ChassisEstimate.speed_y_mps) + 0.5) + 0.22 + (_robotState->auto_slope * 0.2 * (chassisControl.ChassisEstimate.speed_y_mps > 2.05));
	//老电容控制板
	float max_speed_y_rps_error = 0.07 / (0.7 * fabs(chassisControl.ChassisEstimate.speed_y_mps) + 0.5) + 0.22 - (_robotState->auto_slope * 0.2 * (chassisControl.ChassisEstimate.speed_y_mps > 1.95));
//	
//	if(_robotState->auto_slope)
//	{
//		//max_speed_y_rps_error = -0.07 / (0.7 * fabs(chassisControl.ChassisEstimate.speed_y_mps) + 0.2) + 0.4;
//		max_speed_y_rps_error = 0.13 * atan(4 * chassisControl.ChassisEstimate.speed_y_mps - 4.5) + 0.25;
//	}

	switch(_robotState->ctrl_terminal)
	{
		case CONTROL_STOP:
			chassisControl.GimbalCoordinateInput.speed_x_mps = chassisControl.GimbalCoordinateInput.speed_y_mps = 0;
			chassisControl.ChassisFollowControl.revolve_return_flag = 0;
		break;
		
		case CONTROL_FROM_REMOTE:
			/*判断有无操作输入*/	
			if(fabs(_normRemoteCmd->RelativeCH.ch0) < 0.1 && fabs(_normRemoteCmd->RelativeCH.ch1) < 0.1 && fabs(_normRemoteCmd->RelativeCH.ch4) < 0.1)
				is_input_flag = 0;
			
			//有遥控信号的情况下，速度作累加
			if(is_input_flag)
			{
				if(fabs(chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps) < max_speed_x_rps_error)
					chassisControl.GimbalCoordinateInput.speed_x_mps += _normRemoteCmd->RelativeCH.ch0 * 0.015f;
				
				if(fabs(chassisControl.GimbalCoordinateInput.speed_y_mps - chassisControl.ChassisEstimate.speed_y_mps) < max_speed_y_rps_error)
					chassisControl.GimbalCoordinateInput.speed_y_mps += _normRemoteCmd->RelativeCH.ch1 * 0.015f + _robotState->auto_slope * 0.015f + \
																	+ _robotState->auto_slope * 0.07f * (chassisControl.ChassisEstimate.speed_y_mps - 0.25) \
																		* (chassisControl.ChassisEstimate.speed_y_mps > 0.25 && chassisControl.ChassisEstimate.speed_y_mps <= 4.0);
																//	+ _robotState->auto_slope * 0.02525f * (chassisControl.ChassisEstimate.speed_y_mps > 1.15);					
				//额外减速
				if(_normRemoteCmd->RelativeCH.ch0 * chassisControl.ChassisEstimate.speed_x_mps < 0)
				{
					if(chassisControl.GimbalCoordinateInput.speed_x_mps * _normRemoteCmd->RelativeCH.ch0 < 0)
						chassisControl.GimbalCoordinateInput.speed_x_mps = 0;
					chassisControl.GimbalCoordinateInput.speed_x_mps += _normRemoteCmd->RelativeCH.ch0 * 0.01f;
				}
				
				if(_normRemoteCmd->RelativeCH.ch1 * chassisControl.ChassisEstimate.speed_y_mps < 0)
				{				
					if(chassisControl.GimbalCoordinateInput.speed_y_mps * _normRemoteCmd->RelativeCH.ch1 < 0)
						chassisControl.GimbalCoordinateInput.speed_y_mps = 0;
					chassisControl.GimbalCoordinateInput.speed_y_mps += _normRemoteCmd->RelativeCH.ch1 * 0.01f;
				}
				
				chassisControl.GimbalCoordinateInput.speed_x_mps = AbsLimiter(chassisControl.GimbalCoordinateInput.speed_x_mps, max_linear_speed_mps_x * fabs(_normRemoteCmd->RelativeCH.ch0));
				chassisControl.GimbalCoordinateInput.speed_y_mps = AbsLimiter(chassisControl.GimbalCoordinateInput.speed_y_mps, max_linear_speed_mps_y * (fabs(_normRemoteCmd->RelativeCH.ch1) + _robotState->auto_slope));
			}
			//无遥控信号的情况下，记录当前实际速度，但是目标速度给0，速度赋0操作在云台坐标系向底盘坐标系速度转换时进行
			else
			{
				chassisControl.GimbalCoordinateInput.speed_x_mps = chassisControl.ChassisEstimate.speed_x_mps;
				chassisControl.GimbalCoordinateInput.speed_y_mps = chassisControl.ChassisEstimate.speed_y_mps;
			}
		break;

		case CONTROL_FROM_PC:
			if(!_normRemoteCmd->PCKeyBoard.level_key_A && !_normRemoteCmd->PCKeyBoard.level_key_D)
				is_input_flag_x = 0;
			if(!_normRemoteCmd->PCKeyBoard.level_key_W && !_normRemoteCmd->PCKeyBoard.level_key_S && !_normRemoteCmd->PCKeyBoard.level_key_E)
				is_input_flag_y = 0;
			if(is_input_flag_x == 0 && is_input_flag_y == 0)
				is_input_flag = 0;
			
			if(is_input_flag_x)
			{
				if(fabs(chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps) < max_speed_x_rps_error)
					chassisControl.GimbalCoordinateInput.speed_x_mps += (_normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A) * 0.01f;
				
				//额外减速
				if((_normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A) * chassisControl.ChassisEstimate.speed_x_mps < 0)
				{
					if(chassisControl.GimbalCoordinateInput.speed_x_mps * (_normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A) < 0)
						chassisControl.GimbalCoordinateInput.speed_x_mps = 0;
					chassisControl.GimbalCoordinateInput.speed_x_mps += (_normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A) * 0.01f;
				}
				chassisControl.GimbalCoordinateInput.speed_x_mps = AbsLimiter(chassisControl.GimbalCoordinateInput.speed_x_mps, max_linear_speed_mps_x);	
			}
			//无遥控信号的情况下，记录当前实际速度，但是目标速度给0，速度赋0操作在云台坐标系向底盘坐标系速度转换时进行			
			else
				chassisControl.GimbalCoordinateInput.speed_x_mps = chassisControl.ChassisEstimate.speed_x_mps;
			
			if(is_input_flag_y)
			{
				if(fabs(chassisControl.GimbalCoordinateInput.speed_y_mps - chassisControl.ChassisEstimate.speed_y_mps) < max_speed_y_rps_error)
				{   
					if(_robotState->auto_slope == 1)
					{
						chassisControl.GimbalCoordinateInput.speed_y_mps += _robotState->auto_slope * 0.015f + \
																	+ _robotState->auto_slope * 0.07f * (chassisControl.ChassisEstimate.speed_y_mps - 0.25) \
																		* (chassisControl.ChassisEstimate.speed_y_mps > 0.25 && chassisControl.ChassisEstimate.speed_y_mps <= 4.0);
					}
					else
					{
						chassisControl.GimbalCoordinateInput.speed_y_mps += (_normRemoteCmd->PCKeyBoard.level_key_W - _normRemoteCmd->PCKeyBoard.level_key_S) * 0.01f\
																		+ _robotState->auto_slope * 0.005f \
																		+ _robotState->auto_slope * 0.005f * (chassisControl.ChassisEstimate.speed_y_mps - 0.65) \
																		* (chassisControl.ChassisEstimate.speed_y_mps > 0.65 && chassisControl.ChassisEstimate.speed_y_mps <= 1.5)\
																		+ _robotState->auto_slope * 0.0085f * (chassisControl.ChassisEstimate.speed_y_mps > 1.35);
					}
				}
				//额外减速
				if((_normRemoteCmd->PCKeyBoard.level_key_W - _normRemoteCmd->PCKeyBoard.level_key_S) * chassisControl.ChassisEstimate.speed_y_mps < 0)
				{				
					if(chassisControl.GimbalCoordinateInput.speed_y_mps * (_normRemoteCmd->PCKeyBoard.level_key_W - _normRemoteCmd->PCKeyBoard.level_key_S) < 0)
						chassisControl.GimbalCoordinateInput.speed_y_mps = 0;
					chassisControl.GimbalCoordinateInput.speed_y_mps += (_normRemoteCmd->PCKeyBoard.level_key_W - _normRemoteCmd->PCKeyBoard.level_key_S) * 0.01f;
				}				
				chassisControl.GimbalCoordinateInput.speed_y_mps = AbsLimiter(chassisControl.GimbalCoordinateInput.speed_y_mps, max_linear_speed_mps_y);
			}
			//无遥控信号的情况下，记录当前实际速度，但是目标速度给0，速度赋0操作在云台坐标系向底盘坐标系速度转换时进行
			else 
				chassisControl.GimbalCoordinateInput.speed_y_mps = chassisControl.ChassisEstimate.speed_y_mps;				
		break;
	}
	
	//将x,y速度进行归一化，保证合成出的速度向量最大模值和单方向速度向量最大模值相同
	float norm = max_linear_speed_mps_y / sqrt(chassisControl.GimbalCoordinateInput.speed_x_mps * chassisControl.GimbalCoordinateInput.speed_x_mps +\
										chassisControl.GimbalCoordinateInput.speed_y_mps * chassisControl.GimbalCoordinateInput.speed_y_mps);
	if(norm <= 1)
	{
		chassisControl.GimbalCoordinateInput.speed_x_mps *= norm;
		chassisControl.GimbalCoordinateInput.speed_y_mps *= norm;
	}
	
	/*若当前无遥控信号输入，目标速度为0*/
	chassisControl.GimbalCoordinateInput.speed_x_mps *= is_input_flag * is_input_flag_x;
	chassisControl.GimbalCoordinateInput.speed_y_mps *= is_input_flag * is_input_flag_y;	
	//反馈抑制
    if(fabs(gimbalControl.GimbalEstimate.robot_slope_angle) > 10.0f && !_robotState->auto_slope && chassisControl.GimbalCoordinateInput.speed_y_mps == 0)
	{
		chassisControl.GimbalCoordinateInput.speed_x_mps = 1.0 * chassisControl.GimbalCoordinateInput.speed_x_mps \
															+ PIDUpdate(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid),\
															chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps);
		
		chassisControl.GimbalCoordinateInput.speed_y_mps = 1.0 * chassisControl.GimbalCoordinateInput.speed_y_mps +\
															PIDUpdate(&(chassisControl.GimbalCoordinateInput.speed_y_compensate_pid),\
															chassisControl.GimbalCoordinateInput.speed_y_mps - chassisControl.ChassisEstimate.speed_y_mps);
	}
	else
	{
		PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid));
		PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_y_compensate_pid));
	}
	
	if(chassisControl.GimbalCoordinateInput.speed_x_mps == 0)
	{
		chassisControl.GimbalCoordinateInput.speed_x_mps = 1.0 * chassisControl.GimbalCoordinateInput.speed_x_mps \
															+ PIDUpdate(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid),\
															chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps);
	}
	else
	{
		PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid));
	}
	
	#ifdef CENTRIFUGE_REVOLVE		//偏心陀螺，老车不需要
	if(_robotState->chassis_mode == CHASSIS_REVOLVE)
	{
		chassisControl.GimbalCoordinateInput.speed_y_mps += 0.1f * delta_angle_cos;
		chassisControl.GimbalCoordinateInput.speed_x_mps += 0.1f * delta_angle_sin;
	}
	#endif
	
	/*云台坐标系向底盘坐标系转换*/
	chassisControl.ChassisCoordinateInput.speed_x_mps = chassisControl.GimbalCoordinateInput.speed_x_mps * delta_angle_cos \
														+ chassisControl.GimbalCoordinateInput.speed_y_mps * delta_angle_sin;
	chassisControl.ChassisCoordinateInput.speed_y_mps = - chassisControl.GimbalCoordinateInput.speed_x_mps * delta_angle_sin \
														+ chassisControl.GimbalCoordinateInput.speed_y_mps * delta_angle_cos;

	/*-------------------------------------------------------------revolve------------------------------------------------------------*/
	chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.6;

	//50w->0.65 	60w->0.75		70w->0.85	80w->0.95	100w->1.10  	120w->1.25
	if(ext_game_robot_status.chassis_power_limit >= 120)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 1.25f;
	else if(ext_game_robot_status.chassis_power_limit >= 100)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 1.10f;
	else if(ext_game_robot_status.chassis_power_limit >= 80)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.95f;
	else if(ext_game_robot_status.chassis_power_limit >= 70)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.85f;
	else if(ext_game_robot_status.chassis_power_limit >= 60)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.75f;
	else if(ext_game_robot_status.chassis_power_limit >= 50)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.65f;
	
	
	if(_normRemoteCmd->PCKeyBoard.level_key_SHIFT)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps += 0.3;	
	
	if(is_input_flag)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps -= 0.05;
	
	if(_robotState->capacity_mode == NO_CAPACITY)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps -= 0.05;

	//比赛模式下为盲道检录降陀螺转速	
	#ifdef MATCH_MODE 
		static uint16_t match_count = 0;
		if(CONTROL_FROM_REMOTE == _robotState->ctrl_terminal && _robotState->chassis_mode == CHASSIS_REVOLVE)
			match_count = 300;
		if(match_count > 0)
		{
			match_count--;
			chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.2f;
		}
	#endif
	
	
	/*底盘跟随及自旋方向速度输入*/
	switch(_robotState->chassis_mode)
	{
		case CHASSIS_FOLLOW:
		case CHASSIS_FOLLOW_BACK:
			//需要判断此时是刚从自旋变成跟随
			
			//正常的跟随
			if(chassisControl.ChassisFollowControl.revolve_return_flag == 0){

				if(_normRemoteCmd->PCMouse.mouse_right && _upperComputerComm->Receive.aiming_state == 0x33)
					chassisControl.ChassisCoordinateInput.speed_w_rps = PIDUpdate(&(chassisControl.ChassisFollowControl.follow_speed_need_pid),\
																			  chassisControl.ChassisEstimate.chassis_follow_angle_d\
																				+-0.5*(AngleLimit(gimbalControl.GimbalMotorControl.yaw_LTD.x1 - gimbalControl.GimbalEstimate.yaw_angle_d,-180,+180))/*新增陈宝群补偿项，注意符号*/\
																				);
				else
					chassisControl.ChassisCoordinateInput.speed_w_rps = PIDUpdate(&(chassisControl.ChassisFollowControl.follow_speed_need_pid),\
																			  chassisControl.ChassisEstimate.chassis_follow_angle_d\
																				+-0.5*(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d,-180,+180))/*新增陈宝群补偿项，注意符号*/\
																				);
																				}
			//刚从自旋变成跟随
			else
			{	
				/* 切跟随瞬间：yaw目标对齐当前云台实际朝向，用正常跟随逻辑驱动回正 */
				gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;

				chassisControl.ChassisCoordinateInput.speed_w_rps = PIDUpdate(&(chassisControl.ChassisFollowControl.follow_speed_need_pid),\
																			  chassisControl.ChassisEstimate.chassis_follow_angle_d\
																				+-0.5*(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d,-180,+180)));

				if(fabs(AngleLimit(chassisControl.ChassisEstimate.chassis_follow_angle_d, -180, 180)) < 5.0f)
					chassisControl.ChassisFollowControl.revolve_return_flag = 0;
			}	
		break;
		
		case CHASSIS_REVOLVE:
			if((CONTROL_FROM_REMOTE == _robotState->ctrl_terminal || CONTROL_FROM_PC == _robotState->ctrl_terminal) && DT7 == _normRemoteCmd->remote_source)
			{
				chassisControl.ChassisCoordinateInput.speed_w_rps = -chassisControl.GimbalCoordinateInput.max_revolve_speed_rps;//调试限速，赛场上不限速，你妈的记得删
				
				if(_normRemoteCmd->RelativeCH.ch4 < -0.1f){
					// chassisControl.ChassisCoordinateInput.speed_w_rps *= -1;
					}

				chassisControl.ChassisFollowControl.revolve_return_flag = Sign(chassisControl.ChassisEstimate.speed_w_rps);//回正方向
			}
		break;
		
		case CHASSIS_SEPARATE:
			chassisControl.ChassisCoordinateInput.speed_w_rps = 0;
		break;
	}
	/*如果少了轮子，事实上运动学解算也是相同的，
		但是有比较麻烦的力速摩擦等等关系。
		我直接当做黑箱处理然后PID补偿掉*/
	
	/*首先，我简单的想想，掉左边的轮子就是往左边转，所以要往右边补偿，否则反之，大概是前馈？*/
	//chassisControl.ChassisCoordinateInput.speed_w_rps +=
	
	/*或者直行的时候IMU角速度闭环？简单的P控制先 其中chassisControl.ChassisCoordinateInput.compensate_speed_w_dps是error项目*/
	//不对不应该是gimbal yaw
	chassisControl.ChassisEstimate.imu_yaw_dps=gimbalControl.GimbalEstimate.yaw_angular_velocity_dps-yawMotorRec.mechanical_speed_rpm*360/60;//观测器给出的
	chassisControl.ChassisCoordinateInput.compensate_speed_w_dps=(360*chassisControl.ChassisCoordinateInput.speed_w_rps-chassisControl.ChassisEstimate.imu_yaw_dps);//error项
	//chassisControl.ChassisCoordinateInput.speed_w_rps +=chassisControl.ChassisCoordinateInput.compensate_speed_w_dps/360;
	//以上两个方案应该选一个就行0 事实上三轮可以开，那个差角补偿稍微给大一点
	
	//自旋速度限幅
	chassisControl.ChassisCoordinateInput.speed_w_rps = limiter(chassisControl.ChassisCoordinateInput.speed_w_rps, chassisControl.GimbalCoordinateInput.max_revolve_speed_rps);
	//保护赋值
	if(CONTROL_STOP == _robotState->ctrl_terminal)
		chassisControl.ChassisCoordinateInput.speed_w_rps = 0;	

	/*另外的状态机，在吊射模式下防止底盘跟随，速度给0，但是输出不给0，尽量保持静止
	后来发现可以和分离模式耦合*/
	if((_robotState->sniper==SNIPER_ON)){
		chassisControl.ChassisCoordinateInput.speed_x_mps=0;
		chassisControl.ChassisCoordinateInput.speed_y_mps=0;
		chassisControl.ChassisCoordinateInput.speed_w_rps=0;	
				//实则有用	
		//反馈抑制
			if(fabs(gimbalControl.GimbalEstimate.robot_slope_angle) > 10.0f && !_robotState->auto_slope && chassisControl.GimbalCoordinateInput.speed_y_mps == 0)
		{
			chassisControl.GimbalCoordinateInput.speed_x_mps = 1.0 * chassisControl.GimbalCoordinateInput.speed_x_mps \
																+ PIDUpdate(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid),\
																chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps);
			
			chassisControl.GimbalCoordinateInput.speed_y_mps = 1.0 * chassisControl.GimbalCoordinateInput.speed_y_mps +\
																PIDUpdate(&(chassisControl.GimbalCoordinateInput.speed_y_compensate_pid),\
																chassisControl.GimbalCoordinateInput.speed_y_mps - chassisControl.ChassisEstimate.speed_y_mps);
		}
		else
		{
			PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid));
			PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_y_compensate_pid));
		}
	}
}
//
float climb_joint_pos[4]={0.143,-0.1200,-1.13,1.00};
float normal_joint_pos[4]={0.05-0.9416,-0.003114+0.761,0.00819 +0.9416,0.009343-0.761};
static void JointInputUpdate(void)
{
	/* 在此处直接更新关节目标，不再通过数组映射 */
	g_joint_body_target_cmd.roll_d = 0.0f;
	g_joint_body_target_cmd.pitch_d = 0.0f;
	g_joint_body_target_cmd.yaw_d = gimbalPose.yaw_d;
	g_joint_body_target_cmd.roll_rate_dps = 0.0f;
	g_joint_body_target_cmd.pitch_rate_dps = 0.0f;
	g_joint_body_target_cmd.yaw_rate_dps = 0.0f;
	if (_robotState->stand_mode == ROBOT_STAND_MODE_PRE_STAIR)
		g_joint_body_target_cmd.heave_m = 0.3900f;
	else
		g_joint_body_target_cmd.heave_m = 0.3400f;
	g_joint_body_target_cmd.heave_vel_mps = 0.0f;
}
float turnbefore=0;
int turnflag=0;
float turnkp=-0.02;
float turnkd=0.002;
/**
 * @brief 云台相关输入决策更新
 */
float micro_pitch=0;
float micro_yaw=0;
int temp_pitch_count=0,last_temp_pitch=0;
int temp_yaw_count=0,last_temp_yaww=0;
//int spin_flag;
//int last_spin_flag;
static void GimbalInputUpdate(void)//task1,更新机械限位角度
{
	/*物理限位为-23~38度，上下限的offset值为达到机械上下限位时，底盘水平状态下imu的反馈pitch角*/
	const float pitch_upside_limit_offset = 40;
	const float pitch_upside_revolve_limit_offset = 40;	                     //xianfu
	const float pitch_downside_limit_offset = -7;  
	const float small_pitch_upside_limit_offset = 40;
	const float small_pitch_upside_revolve_limit_offset = 40;	 
	const float small_pitch_downside_limit_offset = -7;
	static uint16_t count = 0;
	float smooth=0.02,smoothtry=0.015;
	float micro_change=1;
	extern int com_complet;
	switch(_robotState->ctrl_terminal)
	{
		case CONTROL_STOP: 
			gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
			gimbalControl.GimbalTargetInput.yaw_angular_velocity_dps = 0;
		break;
		//改一下逻辑debug
		case CONTROL_FROM_REMOTE:		
			if((_robotState->sniper==SNIPER_ON)){//task2,修改遥控器pitch输入,小pitch=大pitch输入,其实没什么改得
				if(_robotState->world_enable == WORLD_ENABLE_ON)//ch4>0.1使能世界坐标系yaw目标（上板）
				{
				gimbalControl.GimbalTargetInput.yaw_angle_d=gimbal_yaw_target_rx_d;
				}
				else
				{
				gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 0.1 - _normRemoteCmd->RelativeCH.ch2 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
				}
			}
			if(_robotState->sniper==SNIPER_OFF){
				gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 1.0 - _normRemoteCmd->RelativeCH.ch2 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
			}
			/*计算状态*/		
				if(_robotState->follow==FOLLOW_OFF){
					gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalTargetInput.small_pitch_angle_d;
				}
			#ifndef SHOOT_OFF
			shootControl.ShootEstimate.stir_enableflag_desire = ENABLE; //期望使能
		#else 
			shootControl.ShootEstimate.stir_enableflag_desire = DISABLE; // 期望失能
		#endif
		
		break;	
		case CONTROL_FROM_PC:
		{
			static uint8_t last_aim_mode_state = 0;
			if(_robotState->aim_mode && _robotState->sniper == SNIPER_ON)//C键单次yaw追逐
			{
				gimbalControl.GimbalTargetInput.pitch_angle_d = _upperComputerComm->Receive.target_pitch_angle_d;
				gimbalControl.GimbalTargetInput.yaw_angle_d = gimbal_yaw_target_rx_d;
				c_key_aim_yaw_lock = 1;  /* 锁定yaw目标，禁止ALLHighFreqCal覆盖 */
				robotState.aim_mode = 0;
				last_aim_mode_state = 1;
			}
			else
			{
				uint8_t just_released_c = last_aim_mode_state;
				last_aim_mode_state = 0;
				float temp_pitch = 0;
				float temp_yaw = 0;
				if(_robotState->sniper==SNIPER_OFF){
					smooth=smooth;
				}
				if((_robotState->sniper==SNIPER_ON)){
					int temp_pitch_an = (_normRemoteCmd->PCKeyBoard.level_key_W-_normRemoteCmd->PCKeyBoard.level_key_S);
						/*正在WASD的时候*/
						if (temp_pitch_an !=0){
							last_temp_pitch=temp_pitch_an;
							temp_pitch_count++;
							if(temp_pitch_count>10){
								temp_pitch=(_normRemoteCmd->PCKeyBoard.level_key_W-_normRemoteCmd->PCKeyBoard.level_key_S)*0.01;
								micro_pitch+=(_normRemoteCmd->PCKeyBoard.level_key_W-_normRemoteCmd->PCKeyBoard.level_key_S)*0.01;
							}
						}
					/*狙击WASD结束的时候*/
					if (temp_pitch_an ==0 && last_temp_pitch!=0){
						if(temp_pitch_count<=10){//短按动一格子
							micro_pitch+=last_temp_pitch*0.1;
							temp_pitch +=last_temp_pitch*0.1;
					}
							last_temp_pitch=0;
							temp_pitch_count=0;
					}
					
					/* AD键yaw微调，与WS pitch完全对称 */
					int temp_yaw_an = (_normRemoteCmd->PCKeyBoard.level_key_D-_normRemoteCmd->PCKeyBoard.level_key_A);
					if (temp_yaw_an !=0){
						last_temp_yaww=temp_yaw_an;
						temp_yaw_count++;
						if(temp_yaw_count>10){
							temp_yaw=(_normRemoteCmd->PCKeyBoard.level_key_D-_normRemoteCmd->PCKeyBoard.level_key_A)*0.01;
							micro_yaw+=(_normRemoteCmd->PCKeyBoard.level_key_D-_normRemoteCmd->PCKeyBoard.level_key_A)*0.01;
						}
					}
					if (temp_yaw_an ==0 && last_temp_yaww!=0){
						if(temp_yaw_count<=10){
							micro_yaw+=last_temp_yaww*0.1;
							temp_yaw +=last_temp_yaww*0.1;
						}
						last_temp_yaww=0;
						temp_yaw_count=0;
					}
					
					smooth=smoothtry;
				}
				/*计算状态*/	
				if(_robotState->follow==FOLLOW_OFF){
					if(!just_released_c)
						gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalTargetInput.small_pitch_angle_d;
					if(!(_robotState->sniper == SNIPER_ON && _robotState->mouse_fix == MOUSE_FIX_ON)){
						temp_pitch += SmoothFilterUpdate(&MouseFilterY,_normRemoteCmd->PCMouse.mouse_speed_y)*smooth;
						temp_yaw += SmoothFilterUpdate(&MouseFilterX,_normRemoteCmd->PCMouse.mouse_speed_x)*smooth;
					}
					micro_pitch=0;
					micro_yaw=0;
					/*非锁定状态下鼠标PY可动*/
				}
					//最终整定
					gimbalControl.GimbalTargetInput.small_pitch_angle_d += temp_pitch;	
				extern volatile uint8_t gimbal_yaw_rx_valid;
				if(_robotState->follow!=FOLLOW_ON)//shit
				{
					if (!gimbal_yaw_rx_valid && _robotState->sniper == SNIPER_OFF)
					{
						/* RS485丢失: yaw目标冻结在估计值，开环速度控制见ALLHighFreqCal */
						gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
					}
					else
					{
						gimbalControl.GimbalTargetInput.yaw_angle_d += temp_yaw + micro_yaw;
					}
				}
			}
			/* C键自瞄yaw到位后解除保护锁，恢复ALLHighFreqCal正常覆盖逻辑 */
			if(c_key_aim_yaw_lock && fabs(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d, -180.0f, 180.0f)) < 3.0f)
				c_key_aim_yaw_lock = 0;

		//R键原地转向：仅sniper_off下生效，按下后yaw目标+180°掉头，到位后自动恢复PID
		// if(_robotState->sniper == SNIPER_OFF)
		// {
		// 	static int8_t r_turn_flag = 0;
		// 	if(_normRemoteCmd->PCKeyBoard.level_key_R && r_turn_flag == 0)
		// 	{
		// 		turnbefore = gimbalControl.GimbalEstimate.yaw_angle_d;
		// 		PIDInitialize(&chassisControl.ChassisFollowControl.follow_speed_need_pid, turnkp, 0, turnkd, 0, 3);
		// 		gimbalControl.GimbalTargetInput.yaw_angle_d += 180;
		// 		r_turn_flag = 1;
		// 	}
		// 	else if(!_normRemoteCmd->PCKeyBoard.level_key_R
		// 	        && fabs(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d, -180.0f, 180.0f)) <= 0.5f)
		// 	{
		// 		PIDInitialize(&chassisControl.ChassisFollowControl.follow_speed_need_pid, -0.03, 0, 0.01, 0, 3);
		// 		r_turn_flag = 0;
		// 		turnbefore = gimbalControl.GimbalEstimate.yaw_angle_d;
		// 	}
		// }
		
		#ifndef GIMBAL_OFF
			shootControl.ShootEstimate.stir_enableflag_desire = ENABLE; //期望使能
		#else 
			shootControl.ShootEstimate.stir_enableflag_desire = DISABLE; // 期望失能			
		#endif
		}
		break;
	}
	
	
	
	/*角度限幅*/
	/* yaw内部控制使用连续角，避免±180度处跳变导致“转不动” */
	gimbalControl.GimbalTargetInput.yaw_angle_d = AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d, -180, 180);
//	gimbalControl.GimbalTargetInput.pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.pitch_angle_d, \
//																			 pitch_downside_limit_offset + gimbalControl.GimbalEstimate.robot_slope_angle, 
//																			 pitch_upside_limit_offset + gimbalControl.GimbalEstimate.robot_slope_angle);
	
	if(_robotState->chassis_mode == CHASSIS_REVOLVE || fabs(chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d)>50.0)
	{
			gimbalControl.GimbalTargetInput.pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.pitch_angle_d, \
																			 pitch_downside_limit_offset, \
																			 pitch_upside_revolve_limit_offset);
		gimbalControl.GimbalTargetInput.small_pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.small_pitch_angle_d, \
																			 small_pitch_downside_limit_offset, \
																			 small_pitch_upside_revolve_limit_offset);
	}
	else
	{
		gimbalControl.GimbalTargetInput.pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.pitch_angle_d, \
																			 pitch_downside_limit_offset, \
																			 pitch_upside_limit_offset);//task3,角度限幅
		gimbalControl.GimbalTargetInput.small_pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.small_pitch_angle_d, \
																			 small_pitch_downside_limit_offset, \
																			 small_pitch_upside_limit_offset);
	}
}


/**
 * @brief 射击相关输入决策更新
 */
float targetspeed[30]={0};
extern DataFromJudge bulletSpeed;
float predict_speed0;
float mardio_speed=15.75;
int16_t fric_speed_left_target , fric_speed_right_target,fric_speed_up_target;
int16_t fric_speed_left_target1 , fric_speed_right_target1,fric_speed_up_target1;
float current_fric_speed =4580;// 吊射模式弹速; 3500,4650,dansu,4580-16.77
float default_fric_speed = 3615;//常规模式弹速
float deltaspeed;
float emergesee;
uint8_t stir_flag = 0; 
float temp_angle1;
float temp_angle2;
int stir_cnt=0;
int delay_cnt=0;
int wait_cnt=0;
int during_cnt=0;
uint16_t CRC16_Modbus(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];   // 与当前字节异或

        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
uint8_t frame[8];
/* 拨盘堵转检测与恢复（文件级，ShootInputUpdate/ShootEstimateUpdate/ShootControlUpdate 共用） */
uint16_t stall_count = 0;
static uint8_t stir_stall_recovery_state = 0;  /* 堵转恢复状态: 0=空闲, 1=反转40度中, 2=回预置位中 */
//float current_fric_speed = INITIAL_FRIC_SPEED; 这个之前是注释掉的，看上去下面重复了，估计是调试用的
static void ShootInputUpdate(void)
{
/*----------------------------------------------------stir------------------------------------------------------------*/
	/* 堵转恢复状态机: 0=空闲, 1=反转20度中, 2=回预置位中 */
	static float    stall_preset_target_d = 0.0f;  /* 堵转前的预置位目标 */
	static float    stall_reverse_target_d = 0.0f;  /* 反转20度后的目标 */
	static uint8_t  last_stir_block = 0;
	const float     stall_reverse_angle_d = 20.0f;   /* 反转角度(度) */
	const float     stall_arrive_tolerance_d = 4.0f; /* 到位容差(度) */

	/* 堵转上升沿: 保存预置位, 清除堵转标志, 开始反转20度 */
	if (shootControl.ShootEstimate.stir_block_flag == 1 && last_stir_block == 0 && stir_stall_recovery_state == 0)
	{
		stall_preset_target_d = shootControl.ShootTargetInput.stir_all_target_pos_d;
		stall_reverse_target_d = shootControl.ShootEstimate.stir_all_angle_d + stall_reverse_angle_d;
		shootControl.ShootTargetInput.stir_all_target_pos_d = stall_reverse_target_d;
		stall_count = 0;
		shootControl.ShootEstimate.stir_block_flag = 0;
		stir_stall_recovery_state = 1;  /* 反转中 */
	}
	last_stir_block = shootControl.ShootEstimate.stir_block_flag;

	/* 堵转恢复状态机处理 */
	switch (stir_stall_recovery_state)
	{
	case 1:  /* 反转中: 等待到达反转目标, 若中途再次堵转则放弃反转直接回预置位 */
		if (shootControl.ShootEstimate.stir_block_flag == 1)
		{
			/* 反转途中再次堵转: 放弃反转20度, 立即切回预置位 */
			stall_count = 0;
			shootControl.ShootEstimate.stir_block_flag = 0;
			shootControl.ShootTargetInput.stir_all_target_pos_d = stall_preset_target_d;
			stir_stall_recovery_state = 2;  /* 直接跳到回位中 */
		}
		else if (fabs(shootControl.ShootEstimate.stir_all_angle_d - stall_reverse_target_d) < stall_arrive_tolerance_d)
		{
			/* 反转到位, 设置目标为堵转前的预置位 */
			shootControl.ShootTargetInput.stir_all_target_pos_d = stall_preset_target_d;
			stir_stall_recovery_state = 2;  /* 回位中 */
		}
		break;

	case 2:  /* 回位中: 等待回到预置位, 若堵转则放弃回位就地对齐 */
		if (shootControl.ShootEstimate.stir_block_flag == 1)
		{
			/* 回位途中堵转: 放弃回位, 就地重新对齐六等分点, 结束恢复 */
			stall_count = 0;
			shootControl.ShootEstimate.stir_block_flag = 0;
			stir_stall_recovery_state = 0;  /* 空闲 */
			StirTargetAngleSet();
		}
		else if (fabs(shootControl.ShootEstimate.stir_all_angle_d - stall_preset_target_d) < stall_arrive_tolerance_d)
		{
			/* 回到预置位, 恢复正常, 重新对齐六等分点 */
			stir_stall_recovery_state = 0;  /* 空闲 */
			StirTargetAngleSet();
		}
		break;

	case 0:  /* 空闲 */
	default:
		break;
	}

	uint8_t in_stall_recovery = (stir_stall_recovery_state != 0);

	//uint8_t stir_flag = 0; 变成全局变量
	/*删除了maddaog*/
	/*如果目标角度接近当前角度*/
	/*并且发射脉冲为0，并且 不堵转 或 正在堵转恢复中(恢复期间不发射) */
	//要加新的堵转保护,更严格的限制
		if((_robotState->stir_mode == STIR_ANGLE_CONTROL)\
		&& fabs(shootControl.ShootTargetInput.stir_all_target_pos_d - shootControl.ShootEstimate.stir_all_angle_d) < 5.0f\
		&&stir_flag == 0 && (shootControl.ShootEstimate.stir_block_flag == 0 || in_stall_recovery)\
		/*&&ext_game_robot_status.shooter_barrel_heat_limit - ext_power_heat_data.shooter_42mm_barrel_heat >= 100*/)
	{
		/* 堵转恢复期间不发射, 仅正常模式下发射 */
		if (!in_stall_recovery)
		{
			/*观测值校准*/
			shootControl.ShootEstimate.shoot_count++;//发射计数器增加
			extern void xvni_42_heart_da();
			xvni_42_heart_da();//虚拟热量更新
			shootControl.ShootTargetInput.stir_all_target_pos_d-=60;
			stir_flag=1;//修改发射脉冲，什么玩意啊真几把没用
		}
	}
	else if(_robotState->stir_mode == STIR_LOCK)
		stir_flag = 0;
	
	shootControl.ShootTargetInput.stir_target_pos_rad = shootControl.ShootTargetInput.stir_target_pos / 180.0f * PI;
	shootControl.ShootTargetInput.stir_all_target_pos_rad =shootControl.ShootTargetInput.stir_all_target_pos_d/ 180.0f * PI;

	/* 堵转恢复期间不锁电机, 确保电机可反转和回位 */
	if(_robotState->ctrl_terminal == CONTROL_STOP)
	{
		lock_motor(&hfdcan1,GMJ4310MOTOR_ID);
	}
	else if (in_stall_recovery)
	{
		/* 堵转恢复期间确保电机处于运行状态 */
		if (_stirMotorRec->state == 0)
			start_motor(&hfdcan1, GMJ4310MOTOR_ID);
	}
	else if (shootControl.ShootEstimate.stir_block_flag == 1 && _stirMotorRec->state == 1)
	{
		lock_motor(&hfdcan1,GMJ4310MOTOR_ID);
	}
	else if(_robotState->ctrl_terminal != CONTROL_STOP && shootControl.ShootEstimate.stir_block_flag == 0 && _stirMotorRec->state == 0)
	{
		start_motor(&hfdcan1, GMJ4310MOTOR_ID);
	}
	
	GetStirRealAngle();
}
matrix_t R;
float yaw_out,pitch_out,roll_out;
void Martix_try(){
	  matrix_data_t R_data[9]; 
    matrix_init(&R, 3, 3, R_data);
		create_rotation_matrix_rpy(&R,gimbalControl.GimbalEstimate.yaw_angle_d, gimbalControl.GimbalEstimate.pitch_angle_d, gimbalControl.GimbalEstimate.roll_angle_d);
		extract_euler_angles_rpy(&R, &yaw_out, &pitch_out, &roll_out);
}
void transform_gimbal_to_chassis_pose(
    float imu_yaw, float imu_pitch, float imu_roll,
    float motor_yaw, float motor_pitch,
    float* chassis_yaw, float* chassis_pitch, float* chassis_roll)
{
    // --- 1. 为所有需要的矩阵分配数据缓冲区和实例 ---
    matrix_data_t R_p_w_data[9], R_y_c_data[9], R_p_y_data[9];
    matrix_data_t R_p_y_inv_data[9], R_y_c_inv_data[9];
    matrix_data_t temp_mat_data[9], R_c_w_data[9];

    matrix_t R_p_w, R_y_c, R_p_y;
    matrix_t R_p_y_inv, R_y_c_inv;
    matrix_t temp_mat, R_c_w;

    matrix_init(&R_p_w, 3, 3, R_p_w_data);
    matrix_init(&R_y_c, 3, 3, R_y_c_data);
    matrix_init(&R_p_y, 3, 3, R_p_y_data);
    matrix_init(&R_p_y_inv, 3, 3, R_p_y_inv_data);
    matrix_init(&R_y_c_inv, 3, 3, R_y_c_inv_data);
    matrix_init(&temp_mat, 3, 3, temp_mat_data);
    matrix_init(&R_c_w, 3, 3, R_c_w_data);

    // --- 步骤 A: 计算云台相对于世界(W)的旋转矩阵 R_p_w ---
    // R_p_w = Rz(yaw) * Ry(pitch) * Rx(roll)
    create_rotation_matrix_rpy(&R_p_w, imu_yaw, imu_pitch, imu_roll);

    // --- 步骤 B: 计算电机引入的旋转矩阵 ---
    // Yaw电机(Y)相对于底盘(C)的旋转: 绕Z轴
    // R_y_c = Rz(motor_yaw)
    create_rotation_matrix_rpy(&R_y_c, motor_yaw, 0.0f, 0.0f);

    // Pitch电机(P)相对于Yaw电机(Y)的旋转: 绕Y轴 (根据我们的机械结构约定)
    // R_p_y = Ry(motor_pitch)
    create_rotation_matrix_rpy(&R_p_y, 0.0f, motor_pitch, 0.0f);
    
    // --- 步骤 C: 求解底盘相对于世界的旋转矩阵 R_c_w ---
    // 核心公式: R_c_w = R_p_w * (R_p_y)^-1 * (R_y_c)^-1
    // 利用 R^-1 = R^T
    
    // 计算逆(转置)
    // 使用抽象层的 matrix_inv 宏, 它被定义为 arm_mat_inverse_f32
    // 注意：对于旋转矩阵，转置(transpose)比求逆(inverse)计算量小得多且更稳定。
    // CMSIS-DSP 提供了 arm_mat_trans_f32。我们假设抽象层也提供了 matrix_trans。
    // 如果没有，我们手动实现或使用求逆。这里我们假设求逆是可用的。
    // 如果matrix_inv可用且稳定，可以直接用。
    // arm_mat_inverse_f32(&R_p_y, &R_p_y_inv);
    // arm_mat_inverse_f32(&R_y_c, &R_y_c_inv);
    
    // **更优选择: 使用转置**
    // 假设 `arm_matrix.h` 中有 #define matrix_trans arm_mat_trans_f32
    // 如果没有，我们可以自己实现一个简单的转置函数。
    // 这里我们先用求逆的宏，因为它已经在你的头文件里了。
    matrix_inv(&R_p_y, &R_p_y_inv);
    matrix_inv(&R_y_c, &R_y_c_inv);

    // 进行矩阵链式乘法
    matrix_mult(&R_p_w, &R_p_y_inv, &temp_mat);
    matrix_mult(&temp_mat, &R_y_c_inv, &R_c_w);

    // --- 步骤 D: 从 R_c_w 中提取底盘的欧拉角 ---
    extract_euler_angles_rpy(&R_c_w, chassis_yaw, chassis_pitch, chassis_roll);
}
/*---------------------------------------------------------------------------control task-----------------------------------------------------------------------------------*/
static void ControlInit(void);

static void ChassisEstimateUpdate(void);
static void JointEstimateUpdate(void);
static void GimbalEstimateUpdate(void);
static void ShootEstimateUpdate(void);

static void ChassisControlUpdate(void);//test
static void JointControlUpdate(void);
static void	GimbalControlUpdate(void);
static void ShootControlUpdate(void);

static float ChassisPriorPowerCalc(float* target_motor_mps);
/**
 * @brief 控制任务，完成各电机控制闭环
 */

void ControlTask(void* argument)
{
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count;	
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._control_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._control_task = &this_tick_count;
	
	ControlInit();
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	vTaskDelay(400);
	start_motor(&hfdcan1,GMJ4310MOTOR_ID);
	uint8_t adata[8];
	LK_Motor_run(adata);
	CANTransmit_U8(&hfdcan2, 0x141 , adata);
	CANTransmit_U8(&hfdcan3, 0x142,  adata);
	vTaskDelay(200);
	while(1)
	{//kkg

		/*任务主进程*/
		//观测
		ChassisEstimateUpdate();
		JointEstimateUpdate();
		GimbalEstimateUpdate();
		ShootEstimateUpdate();

		//闭环
		ChassisControlUpdate();
		GimbalControlUpdate();
		ShootControlUpdate();
		JointControlUpdate();
		
		//发送指令
		MotorControlCANSend();
		
		
		

		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
		vTaskDelayUntil(&current_tick_count, CONTROL_TASK_PERIOD_SET);
	}
}

/**
 * @brief 控制执行机构相关初始化
 */
float kpfric=17.5;       //hxgpid
float kdfric=12.5;
static void ControlInit(void)
{
//----------------------------------------底盘电机初始化----------------------------------------------------------------------------------------------------------------------
	//底盘电机//kp 15 kd 5
	PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[LF]), 8, 0, 2.0, 0, M3508_MAX_OUTPUT_CURRENT);
	PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[RF]), 8, 0, 2.0, 0, M3508_MAX_OUTPUT_CURRENT);
	PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[RB]), 7, 0, 1.5, 0, M3508_MAX_OUTPUT_CURRENT);
	PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[LB]), 7, 0, 1.5, 0, M3508_MAX_OUTPUT_CURRENT);
	float pitch_angle_offset_d = 4.9;
	gimbalControl.GimbalTargetInput.pitch_angle_d = pitch_angle_offset_d;
	float h_temp = CONTROL_TASK_PERIOD_SET / 1000.0;
//---------------------------------------pitch-LADRC初始化-------------------------------------------------------------------------------------------------------------------------
	//r大响应快,但是可能超调,发散,r小反应慢
	float pitch_eso_wo=20.0f,pitch_eso_b0 = 3.0f,pitch_z3_limit = 2000.0f,e_min=0,e_max=0;//增大带宽噪声不好,使z3跳跳
	gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1_min=-15;
	gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1_max=+36;
	gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2_min=-200;
	gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2_max=+200;//不要限幅,会怪怪的
	LTDInitialize(&gimbalControl.GimbalMotorControl.pitch_LTD, 20, 0.003, -30, 60);//pitch,ltd调r
	ESOInitialize(&gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso, h_temp,
	   	pitch_eso_b0,
		3.0f*pitch_eso_wo,
		3.0f*pitch_eso_wo*pitch_eso_wo,
		0.30f*pitch_eso_wo*pitch_eso_wo*pitch_eso_wo,//如果只降低z3积分可以减少噪声,相当于滤波?
	pitch_z3_limit, e_min, e_max);
	LESFInitialize(&gimbalControl.GimbalMotorControl.pitch_angle_adrc.esf, 0.0f, 70.0f, 8.0f, 50.0f, 1000.0f);
//-----------------------------------------yaw控制初始化---------------------------------------------------------------------------------------------------
	LTDInitialize(&gimbalControl.GimbalMotorControl.yaw_LTD,20 , 0.003, -180, 180);
	//位置环pi,速度环pd
	PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_pos_pid, 8.0, 0.09, 0, 100, 40);//6.0,0.09,0,100,40
	PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_speed_pid, 0.075, 0, 0.010, 0, 5);//0.065,0,0.01,0,3
	gimbalControl.GimbalEstimate.shoot_window_flag = 0.0f;
	gimbalControl.GimbalMotorControl.yaw_window_tff=0;
//--------------------------------------------拨盘-----------------------------------------------------------------------------------------------------------------------
	shootControl.ShootMotorControl.stir_preset_angle = STIR_PRESET_ANGLE;//预置位
	shootControl.ShootTargetInput.stir_target_vol = STIR_MAX_SPEED;
	shootControl.ShootEstimate.stir_enableflag_desire = DISABLE;
//----------------------------------------功率预测使用的滤波器---------------------------------------------------------------------------------------------------------------
	AverageFilterInitialize(&PowerFilter);
//----------------------------------------------关节电机初始化----------------------------------------------------------------
    JointForceControlInit(30,0.003);
	JointForceControlTuningParamInit();//此处调参输入接口kkg
	}
float see_error1,see_error2,see_error3;
float w_d2=0;
float k_ff=2;
int temp_yaw=0,last_temp_yaw=0;
//float 
float wd3=3,wd3p=30;
/*新云台控制函数*/
extern DJIGMotorRec smallpitchMotorRec;
extern Pose gimbalPose;
int finish_flag=0;
int target_torque=0;
float w_d;
int last_u;
int fl_u;
int shoot_flag;
int shoot_cnt;
float sniper_pitch_angle;
float angle_control;
float lowpass_pitch=0;
extern int64_t circle_angle;
extern float pitch_angle_from_match;
float torque_out;
float angle_error(float target, float current)
{
    float err = target - current;

    while(err > 180)  err -=360;
    while(err < -180) err +=360;

    return err;
}//反转处理函数
float yaw_speed_kp = -0.04;
float yaw_pos_kp = 5.0f;  /* 位置环增益: 角度误差(°) × K → 期望速度(°/s) */
void ALLHighFreqCal(void)
{
		/* 发射窗口检测: 发射后延时60ms进入窗口, 窗口保持40ms */
		static uint16_t last_shoot_count = 0;
		static float shoot_window_delay_ms = 0.0f;
		static float shoot_window_remain_ms = 0.0f;
		const float shoot_window_shift_ms = 70.0f;
		const float shoot_window_hold_ms = 70.0f;
		if(shootControl.ShootEstimate.shoot_count != last_shoot_count)
		{
			shoot_window_delay_ms = shoot_window_shift_ms;
			shoot_window_remain_ms = 0.0f;
			last_shoot_count = shootControl.ShootEstimate.shoot_count;
		}
		if(shoot_window_delay_ms > 0.0f)
		{
			shoot_window_delay_ms -= (float)CONTROL_TASK_PERIOD_SET;
			if(shoot_window_delay_ms <= 0.0f)
			{
				shoot_window_remain_ms = shoot_window_hold_ms;
			}
			gimbalControl.GimbalEstimate.shoot_window_flag = 0.0f;
		}
		else if(shoot_window_remain_ms > 0.0f)
		{
			shoot_window_remain_ms -= (float)CONTROL_TASK_PERIOD_SET;
			gimbalControl.GimbalEstimate.shoot_window_flag = 1.0f;
		}
		else
		{
			gimbalControl.GimbalEstimate.shoot_window_flag = 0.0f;
		}
//---------------------------------------------yaw轴控制,MIT&IMU和编码器--------------------------------------------------------------------------------------------------
		float yaw_pos_err_d = AngleLimit(gimbalControl.GimbalMotorControl.yaw_LTD.x1 - gimbalControl.GimbalEstimate.yaw_angle_d, -180.0f, 180.0f);
		float pre_yaw_Tff;
		if(fabs(yaw_pos_err_d) < 0.4f)//0.4 
		{
			gimbalControl.GimbalMotorControl.yaw_pos_pid.ki=0.12;//0.11
			gimbalControl.GimbalMotorControl.yaw_pos_pid.kp=8.4;//7.5
			gimbalControl.GimbalMotorControl.yaw_speed_pid.kp=0.068;//0.06
			pre_yaw_Tff=gimbalControl.GimbalMotorControl.yaw_LTD.x2*0.0;
		}
		else 
		{
			gimbalControl.GimbalMotorControl.yaw_pos_pid.ki=0;
			gimbalControl.GimbalMotorControl.yaw_pos_pid.sum_error=0;
			pre_yaw_Tff=gimbalControl.GimbalMotorControl.yaw_LTD.x2*0.05;
		}
		LTDUpdate(&gimbalControl.GimbalMotorControl.yaw_LTD, gimbalControl.GimbalTargetInput.yaw_angle_d);
		PIDUpdate(&gimbalControl.GimbalMotorControl.yaw_pos_pid, yaw_pos_err_d);
		static float yaw_pos_out_lpf = 0.0f;
		const float yaw_pos_lpf_alpha = 0.3f;
		float yaw_pos_out_raw = gimbalControl.GimbalMotorControl.yaw_pos_pid.output;
		yaw_pos_out_lpf = yaw_pos_lpf_alpha * yaw_pos_out_raw + (1.0f - yaw_pos_lpf_alpha) * yaw_pos_out_lpf;
		PIDUpdate(&gimbalControl.GimbalMotorControl.yaw_speed_pid, yaw_pos_out_lpf - gimbalControl.GimbalEstimate.yaw_angular_velocity_dps);
		gimbalControl.GimbalMotorControl.yaw_target_output=gimbalControl.GimbalMotorControl.yaw_speed_pid.output + pre_yaw_Tff;	
		static float yaw_speed_kd = 0.0f;
		static float last_speed_err = 0.0f;
		static float yaw_sniper_i_sum = 0.0f;
		//力矩方向:逆时针是正,顺时针负.大概零点几
		//x1:初始位置顺时针是正
		//x2(速度方向):逆时针是负,顺时针是正,大概十几,几十	
		//假设向顺时针方向旋转,pos_error是正,pos_output是正值对的,speed_output也是正,实际应该给负值
	if(CONTROL_STOP != _robotState->ctrl_terminal){
		
		if(_robotState->sniper == SNIPER_ON)
		{
			const float yaw_window_tff_mag = 0.28f;
			const float yaw_window_angle_deadband_deg = 0.017f;//16ms数据0.28f,0.017f
			float yaw_window_tff = 0.0f;
			if(gimbalControl.GimbalEstimate.shoot_window_flag > 0.5f)
			{
				/* 方向由角度误差决定: target - measure */
				float yaw_window_err = AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d, -180, 180);
				if(yaw_window_err > yaw_window_angle_deadband_deg)
					yaw_window_tff = +yaw_window_tff_mag;
				else if(yaw_window_err < -yaw_window_angle_deadband_deg)
					yaw_window_tff = -yaw_window_tff_mag;
				else
					yaw_window_tff = 0.0f;
			}
			gimbalControl.GimbalMotorControl.yaw_window_tff = yaw_window_tff;
			gimbalControl.GimbalMotorControl.mit_p   = 0;
			gimbalControl.GimbalMotorControl.mit_v   = 0;
			gimbalControl.GimbalMotorControl.mit_Kp  = 0;
			gimbalControl.GimbalMotorControl.mit_Kd  = 0;
			gimbalControl.GimbalMotorControl.mit_Tff = AbsLimiter(-(gimbalControl.GimbalMotorControl.yaw_speed_pid.output + pre_yaw_Tff), 5.0f);
			//gimbalControl.GimbalMotorControl.mit_Tff=0;
			last_speed_err = 0.0f;
		}
		else
		{
			/* 普通模式 */
			extern volatile uint8_t gimbal_yaw_rx_valid;
			if (!gimbal_yaw_rx_valid)
			{
				/* RS485丢失 → 开环MIT速度控制: 鼠标直接映射为yaw转速 */
				float cmd_vel_dps = (float)_normRemoteCmd->PCMouse.mouse_speed_x * 0.03f;

				gimbalControl.GimbalMotorControl.mit_p   = DMyawMotorRec.pos_d;
				gimbalControl.GimbalMotorControl.mit_v   = cmd_vel_dps;
				gimbalControl.GimbalMotorControl.mit_Kp  = 0.5f;
				gimbalControl.GimbalMotorControl.mit_Kd  = 0.3f;
				gimbalControl.GimbalMotorControl.mit_Tff = 0.0f;
				gimbalControl.GimbalMotorControl.yaw_window_tff = 0.0f;
				gimbalControl.GimbalMotorControl.yaw_LTD.x2 = 0;
				gimbalControl.GimbalMotorControl.yaw_LTD.x1 = 0;
				last_speed_err = 0.0f;
				yaw_sniper_i_sum = 0.0f;
			}
			else
			{
			/* 普通模式: 纯力矩控制 */
			float yaw_err_deg = AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d, -180, 180);
			
			extern volatile float gimbal_yaw_dps_rx;
			extern float yaw_speed_kp;
			extern float yaw_pos_kp;
			float speed_err = yaw_err_deg * yaw_pos_kp - gimbal_yaw_dps_rx; /* 期望速度 - 反馈速度 */
			torque_out = yaw_speed_kp * speed_err + yaw_speed_kd * (speed_err - last_speed_err);
			last_speed_err = speed_err;
			gimbalControl.GimbalMotorControl.mit_p   = 0;
			gimbalControl.GimbalMotorControl.mit_v   = 0;
			gimbalControl.GimbalMotorControl.mit_Kp  = 0;
			gimbalControl.GimbalMotorControl.mit_Kd  = 0;
			gimbalControl.GimbalMotorControl.mit_Tff = AbsLimiter(torque_out, 10.0f);
			gimbalControl.GimbalMotorControl.yaw_window_tff = 0.0f;
			gimbalControl.GimbalMotorControl.yaw_LTD.x2=0;
			gimbalControl.GimbalMotorControl.yaw_LTD.x1=0;
			yaw_sniper_i_sum = 0.0f;
			}
		}

	}
	else
	{
			gimbalControl.GimbalMotorControl.mit_p = DMyawMotorRec.pos_d;
			gimbalControl.GimbalMotorControl.mit_v = 0.0f;
			gimbalControl.GimbalMotorControl.mit_Kp = 0.0f;
			gimbalControl.GimbalMotorControl.mit_Kd = 0.0f;
			gimbalControl.GimbalMotorControl.mit_Tff = 0.0f;
			gimbalControl.GimbalMotorControl.yaw_window_tff = 0.0f;
			last_speed_err = 0.0f;
			yaw_sniper_i_sum = 0.0f;
			yaw_pos_out_lpf = 0.0f;
	}
//------------------------------------------------------------------------------------------------------------------------------------------------
	extern uint8_t shit_delay_count;
	/*共同保护*/
	static uint8_t last_ctrl_terminal = CONTROL_STOP;
	if(last_ctrl_terminal == CONTROL_STOP && _robotState->ctrl_terminal != CONTROL_STOP)
	{
		/* 退保护边沿：对齐目标和跟踪器状态，避免上电瞬间冲击 */
		gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
		gimbalControl.GimbalMotorControl.yaw_LTD.x1 = gimbalControl.GimbalEstimate.yaw_angle_d;
		gimbalControl.GimbalMotorControl.yaw_LTD.x2 = 0.0f;
		gimbalControl.GimbalMotorControl.yaw_LTD.error_sum = 0.0f;
		gimbalControl.GimbalMotorControl.mit_p = DMyawMotorRec.pos_d;
		gimbalControl.GimbalMotorControl.mit_v = 0.0f;
		gimbalControl.GimbalMotorControl.mit_Kp = 0.0f;
		gimbalControl.GimbalMotorControl.mit_Kd = 0.0f;
		gimbalControl.GimbalMotorControl.mit_Tff = 0.0f;
		shit_delay_count = 0;
	}
	last_ctrl_terminal = _robotState->ctrl_terminal;

	if((CONTROL_STOP == _robotState->ctrl_terminal||shit_delay_count<30) && !c_key_aim_yaw_lock)
	{

		gimbalControl.GimbalTargetInput.yaw_angle_d=gimbalControl.GimbalEstimate.yaw_angle_d;
		
		gimbalControl.GimbalMotorControl.pitch_LTD.error_sum=0;
		gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1=0;
		gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2=0;
		gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z3=0;
		gimbalControl.GimbalMotorControl.pitch_angle_adrc.esf.output=0;
		gimbalControl.GimbalMotorControl.pitch_angle_adrc.u_0=0;
		gimbalControl.GimbalMotorControl.pitch_angle_adrc.u=0;
		
		gimbalControl.GimbalMotorControl.yaw_LTD.error_sum=0;
		gimbalControl.GimbalMotorControl.small_pitch_target_output = gimbalControl.GimbalMotorControl.small_pitch_target_output = 0;
	}
}
//------------------------------------------------------------------------------------------------------------------------------------------------	
/**
 * @brief 底盘相关观测数据更新
 */
static void ChassisEstimateUpdate(void)
{
	/*底盘云台的相对角度（减去正前方偏置，与yaw_enc_deg保持一致）*/
	extern float yaw_dm_forward_offset_rad;
	chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d = (yaw_dm_forward_offset_rad - _DMyawMotorRec->pos_d) * (180.0f/3.141592f);
	chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d = AngleLimit(chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d, -180, 180);
	/*跟随角度 = 云台与底盘的相对角，云台正前方时为0*/
	chassisControl.ChassisEstimate.chassis_follow_angle_d = chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d;														  //+ (- gimbalControl.GimbalMotorControl.yaw_angle_adrc.esf.e_1);
	/* CHASSIS_FOLLOW_BACK: 跟随方向反转180度，机体背向云台朝向 */
	if (_robotState->chassis_mode == CHASSIS_FOLLOW_BACK)
	{
		chassisControl.ChassisEstimate.chassis_follow_angle_d += 180.0f;
	}
  
	static uint16_t shake_count = 0;
	//唉，为什么这里一点那里一点啊,我也懒得给你重构了，加史
	if(_robotState->sniper==SNIPER_OFF)
	if(_normRemoteCmd->PCKeyBoard.level_key_CTRL)
		shake_count = 400;
	if (shake_count > 0)
	{	
		shake_count--;
		switch (shake_count / 50)
		{
			case 8: case 7: case 5: case 3: case 1: chassisControl.ChassisEstimate.chassis_follow_angle_d += 45;break;
			default: chassisControl.ChassisEstimate.chassis_follow_angle_d -= 45;
		}
	}
	
	chassisControl.ChassisEstimate.chassis_follow_angle_d = AngleLimit(chassisControl.ChassisEstimate.chassis_follow_angle_d, -180, 180);
	
	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
		chassisControl.ChassisEstimate.wheel_real_speed_mps[i] = -_chassisMotorRec[i].mechanical_speed_rpm * WHEEL_RPM_TO_WHEEL_MPS;
	
	/*根据底盘电机安装方向，逆运动学解算出当前底盘的真实速度*/
	chassisControl.ChassisEstimate.speed_w_rps = (chassisControl.ChassisEstimate.wheel_real_speed_mps[LF] + chassisControl.ChassisEstimate.wheel_real_speed_mps[RF]\
												 +chassisControl.ChassisEstimate.wheel_real_speed_mps[LB] + chassisControl.ChassisEstimate.wheel_real_speed_mps[RB])\
												 / 4.0f * WHEEL_MPS_TO_ROBOT_RPS;
	
	float chassis_coordinate_speed_x_mps = (chassisControl.ChassisEstimate.wheel_real_speed_mps[LF] + chassisControl.ChassisEstimate.wheel_real_speed_mps[RF]\
										 - chassisControl.ChassisEstimate.wheel_real_speed_mps[LB] - chassisControl.ChassisEstimate.wheel_real_speed_mps[RB])\
										 / 4.0f / 1.414f;
	
	float chassis_coordinate_speed_y_mps = (chassisControl.ChassisEstimate.wheel_real_speed_mps[LF] - chassisControl.ChassisEstimate.wheel_real_speed_mps[RF]\
										 + chassisControl.ChassisEstimate.wheel_real_speed_mps[LB] - chassisControl.ChassisEstimate.wheel_real_speed_mps[RB])\
										 / 4.0f / 1.414f;

	float delta_angle_cos, delta_angle_sin;
	arm_sin_cos_f32(chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d, &delta_angle_sin, &delta_angle_cos);	
	chassisControl.ChassisEstimate.speed_x_mps = chassis_coordinate_speed_x_mps * delta_angle_cos \
											   - chassis_coordinate_speed_y_mps * delta_angle_sin;
	chassisControl.ChassisEstimate.speed_y_mps = chassis_coordinate_speed_x_mps * delta_angle_sin \
											   + chassis_coordinate_speed_y_mps * delta_angle_cos;
}

//电机结构体1--2
//         |  |
//         4--3
static void JointEstimateUpdate(void)
{
	JointBodyState body_state_for_height = {0};
	float body_height_m_obs = jointControl.JointEstimate.body_height_m;
	float body_height_vel_mps_obs = jointControl.JointEstimate.body_height_vel_mps;
	float joint_vel_radps_raw[4];

	for(uint8_t i = 0; i < 4; i++)//更新数据
	{
		jointControl.JointEstimate.frame_counter[i] = _jointMotorEec[i].frame_counter;
		jointControl.JointEstimate.id[i] = (uint32_t)_jointMotorEec[i].id;
		jointControl.JointEstimate.state[i] = _jointMotorEec[i].state;
		jointControl.JointEstimate.pos_d[i] = _jointMotorEec[i].pos_d*180/3.141592f;//注意原来是弧度
		jointControl.JointEstimate.vel_radps[i] = _jointMotorEec[i].vel_radps*180/3.141592f;
		joint_vel_radps_raw[i] = _jointMotorEec[i].vel_radps;
		jointControl.JointEstimate.toq[i] = _jointMotorEec[i].toq;//反馈扭矩
		jointControl.JointEstimate.Kp[i] = _jointMotorEec[i].Kp;
		jointControl.JointEstimate.Kd[i] = _jointMotorEec[i].Kd;
		jointControl.JointEstimate.Tmos[i] = _jointMotorEec[i].Tmos;
		jointControl.JointEstimate.Tcoil[i] = _jointMotorEec[i].Tcoil;
	}
	float motor_angles_rad[4];
	float feedback_pos_rad[4];
	for(uint8_t i = 0; i < 4; i++)
	{
		feedback_pos_rad[i] = _jointMotorEec[i].pos_d;
	}
	JointBuildMotorAnglesRadFromFeedback(feedback_pos_rad, motor_angles_rad);
	for(uint8_t i = 0; i < 4; i++)
	{
		jointControl.JointEstimate.motor_angles_rad[i] = motor_angles_rad[i];
	}
	//更新输入角
	JointForceControlSetMotorAngleRad(motor_angles_rad);
	//更新接触点在机体系中的位置
	JointUpdateLegPoseFromMotorAngle();
	//更新雅可比矩阵
	JointUpdateLegJacobiansFromMotorAngle();
	//更新接触检测
	JointForceControlSetContact(jointControl.JointEstimate.frame_counter,
					jointControl.JointEstimate.toq,
					(_robotState->ctrl_terminal != CONTROL_STOP) ? 1u : 0u);

	/* 观测态由 gimbalPose 提供，角速度先置 0（后续可替换为滤波角速度） */
	g_joint_body_state_obs.pitch_d = gimbalPose.pitch_d;
	g_joint_body_state_obs.yaw_d = gimbalPose.yaw_d;
	g_joint_body_state_obs.roll_d = gimbalPose.roll_d;
	g_joint_body_state_obs.roll_rate_dps = gimbalPose.roll_radps*57.29578f;
	g_joint_body_state_obs.pitch_rate_dps = gimbalPose.pitch_radps*57.29578f;
	g_joint_body_state_obs.yaw_rate_dps =gimbalPose.yaw_radps*57.29578f;
	g_joint_body_state_obs.accel_x = gimbalPose.accel_x;
	g_joint_body_state_obs.accel_y = gimbalPose.accel_y;
	g_joint_body_state_obs.accel_z = gimbalPose.accel_z;

	JointForceControlConvertBodyState(&g_joint_body_state_obs, &body_state_for_height);
	//更新机体高度与竖直速度观测（世界系 z）
	JointEstimateBodyHeightVelocity(&body_state_for_height,
				       joint_vel_radps_raw,
				       &body_height_m_obs,
				       &body_height_vel_mps_obs);
	jointControl.JointEstimate.body_height_m = body_height_m_obs;
	jointControl.JointEstimate.body_height_vel_mps = body_height_vel_mps_obs;

	/* 上台阶检测：仅在 PRE_STAIR 模式下使能，NORMAL / PRE_DOWN_STAIR 模式清除状态 */
	if (_robotState->stand_mode == ROBOT_STAND_MODE_PRE_STAIR)
	{
		JointStairUpDetect();
	}
	else if (_robotState->stand_mode == ROBOT_STAND_MODE_NORMAL
	         || _robotState->stand_mode == ROBOT_STAND_MODE_PRE_DOWN_STAIR)
	{
		JointStairUpDetectReset();
	}
	if (JointStairUpIsDetected())
	{
		robotState.stand_mode = ROBOT_STAND_MODE_STAIR_UP;
	}
}//代办:检查速度映射

/**
 * @brief 云台相关观测数据更新
 */
static void GimbalEstimateUpdate(void)
{
	/*云台位姿信息在imu_task中更新*/
	const uint16_t pitch_offset_d = PITCH_OFFSET_MACHENICAL_ANGLE;//p轴在水平位置时电机的机械角（度），需要测,task4测量实际机械角
	//底盘和p轴视作刚体，当底盘倾斜，p轴定子随底盘倾斜，云台受imu反馈控制保持一个角度时，此时电机返回机械角和底盘不倾斜时返回机械角是存在差值的，该差值即为倾斜角
	//1.具体计算方法为：记录p轴电机机械角在底盘水平的情况下，控制云台达到imu反馈0度角时的机械角偏置值；
	//2.实时使用p轴电机机械角与该偏置值相减并转成角度制后，即为假设机器人底盘水平状态下的p轴角度；
	//3.以该角度值与当前imu反馈的pitch轴角度相减，便可得到实际机器人底盘倾斜角度
	//note:规定底盘向上倾斜为正，向下倾斜为负，计算过程中需要根据实际情况添置正负号
	float pitch_angle_offset_d =(_pitchMotorRec->mechanical_angle - pitch_offset_d) * 360.0f /LK_FULL_CIRCLE_MECHENICAL_ANGLE;//符号注意，记得改这玩意
	pitch_angle_offset_d = AngleLimit(pitch_angle_offset_d, -180, 180);
	gimbalControl.GimbalEstimate.robot_slope_angle = pitch_angle_offset_d + gimbalControl.GimbalEstimate.pitch_angle_d;
	gimbalControl.GimbalEstimate.robot_slope_angle = AngleLimit(gimbalControl.GimbalEstimate.robot_slope_angle, -180, 180);
}

/// @brief 云台观测位姿更新，该更新周期与imu_task任务更新位姿周期一致，故需外部调用该更新函数
/// @param pitch_angle 观测当前pitch轴角，单位degree
/// @param pitch_angle_w 观测当前pitch转角方向角速度，单位rad/s
/// @param yaw_angle  观测当前yaw轴角，单位degree
/// @param yaw_angle_w 观测当前yaw转角方向角速度，单位rad/s
/// @note 角速度仍有高频噪声，需经过平滑滤波
/* DM yaw编码器在“云台正前方”时的机械角(rad)，由debug实测后填写 */
float yaw_dm_forward_offset_rad = -2.54f;
uint8_t shit_delay_count=0;
float shit_temp_pitch_comp=0;
void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w,float roll_angle,float roll_angle_w)
{
	gimbalControl.GimbalEstimate.roll_angle_d=roll_angle;
	gimbalControl.GimbalEstimate.roll_angular_velocity_dps=roll_angle_w;
	pitch_angle_from_match= (_pitchMotorRec->mechanical_angle - PITCH_OFFSET_MACHENICAL_ANGLE) * 360.0f /LK_FULL_CIRCLE_MECHENICAL_ANGLE;
	pitch_angle_from_match= AngleLimit(pitch_angle_from_match, -180, 180);

	/*=== yaw编码器角度（DM电机）===
	 * 数据流：DMyawMotorRec.pos_d(rad) -> 减offset -> 转degree -> yaw_enc_deg
	 * 用途1: 作为GimbalEstimate.yaw_angle_d的备用来源（RS485 IMU优先）
	 * 用途2: 差分得到yaw角速度的备用来源
	 */
	static uint8_t yaw_speed_inited = 0;
	static float last_yaw_enc_deg = 0.0f;
	static float yaw_speed_lpf_dps = 0.0f;
	const float dt = IMU_TASK_PERIOD_SET / 1000.0f;

	float yaw_enc_deg = AngleLimit((DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f, -180.0f, 180.0f);

	if(!yaw_speed_inited)
	{
		last_yaw_enc_deg = yaw_enc_deg;
		yaw_speed_inited = 1;
	}
	{
		const float yaw_delta_deg = AngleLimit(yaw_enc_deg - last_yaw_enc_deg, -180.0f, 180.0f);
		const float yaw_speed_raw_dps = yaw_delta_deg / dt;
		yaw_speed_lpf_dps = 0.2f * yaw_speed_raw_dps + 0.8f * yaw_speed_lpf_dps;//速度低通滤波
		last_yaw_enc_deg = yaw_enc_deg;
	}
	extern volatile float gimbal_yaw_rx_d;
	extern volatile float gimbal_yaw_dps_rx;
	extern volatile uint8_t gimbal_yaw_rx_valid;
	if (_robotState->sniper == SNIPER_OFF) 
	{
		gimbalControl.GimbalEstimate.yaw_angle_d = gimbal_yaw_rx_d;
		gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = gimbal_yaw_dps_rx;
	} 
	else 
	{
		gimbalControl.GimbalEstimate.yaw_angle_d = yaw_enc_deg;
		gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = gimbal_yaw_dps_rx;//也用角速度imu
	}
	if(_robotState->sniper==SNIPER_OFF )
		gimbalControl.GimbalEstimate.pitch_angle_d = pitch_angle+shit_temp_pitch_comp;
	if(_robotState->sniper==SNIPER_ON )
		gimbalControl.GimbalEstimate.pitch_angle_d=pitch_angle_from_match;
	gimbalControl.GimbalEstimate.pitch_angular_velocity_dps = pitch_angle_w * 57.3f;

	if(shit_delay_count<200)
		shit_delay_count++;
	if(lastRobotState.lens != _robotState->sniper)
	{
		shit_delay_count = 0;
		gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
		lastRobotState.lens = _robotState->sniper;
	}
	transform_gimbal_to_chassis_pose(gimbalControl.GimbalEstimate.yaw_angle_d,gimbalControl.GimbalEstimate.pitch_angle_d,gimbalControl.GimbalEstimate.roll_angle_d,\
		DMyawMotorRec.p_int*360/65535,pitch_angle_from_match,&chassisControl.ChassisEstimate.yaw_d,\
		&chassisControl.ChassisEstimate.pitch_d,&chassisControl.ChassisEstimate.roll_d);
}

/**
 * @brief 发射相关观测更新
 */
static void ShootEstimateUpdate(void)
{
	/*拨盘电机使能失能认定*/
	static uint8_t disbuf = 0;
	static uint8_t enablebuf = 0;
	static uint16_t stir_state_detect_counter = 0;
	stir_state_detect_counter++;

	/*新堵转检测*/
	static uint8_t reverse_count = 0;//延迟恢复计数
	//static uint16_t stall_count = 0;//堵转时间计数
	//if(fabs(_stirMotorRec->vel_radps) < STIR_CAUTION_SPEED && fabs(_stirMotorRec->toq) > 5.0f)
	if(fabs(_stirMotorRec->vel_radps) < STIR_CAUTION_SPEED && fabs(_stirMotorRec->toq) > 8.0f)
		stall_count+=10;
	else if(stall_count>0)
		stall_count--;
	
	if(stall_count>=500)//400
		shootControl.ShootEstimate.stir_block_flag = 1;      //1
	if(stall_count==0)
		shootControl.ShootEstimate.stir_block_flag = 0;
	/*新拨盘自动预制 退保护就转拨盘*/
	
//	/*拨盘自动预置*/
	static uint8_t last_fric_state, cur_fric_state, last_robot_state, cur_robot_state;
	cur_fric_state = _robotState->fric_mode;
	cur_robot_state = _robotState->ctrl_terminal;
//	if((last_fric_state == 0 && cur_fric_state) || (last_robot_state == 0 && cur_robot_state))
	if(last_robot_state == 0 && cur_robot_state && _stirMotorRec->frame_counter)
	{
		StirTargetAngleSet();
	}
	last_fric_state = cur_fric_state;
	last_robot_state = cur_robot_state;

	/*10s无操作自动校准*/
	static uint16_t calibration_count = 0;
	if(_stirMotorRec->vel_radps < STIR_CAUTION_SPEED && _robotState->ctrl_terminal != CONTROL_STOP && !shootControl.ShootEstimate.stir_reset_flag)
	{
		calibration_count++;
		if(calibration_count % 1000 == 0)
			shootControl.ShootEstimate.stir_real_angle = shootControl.ShootTargetInput.stir_target_pos;//?
	}
	else
		calibration_count = 0;
	
	static uint16_t last_count = 0, last_last_count = 0, cur_count, time_count = 0;
	time_count++;
	cur_count = _stirMotorRec->frame_counter;
	if(time_count % 100 == 0)
		{/*连这三个周期不变的话，认定为拨盘下电*/
		if(last_last_count == last_count && last_count == cur_count && !shootControl.ShootEstimate.stir_reset_flag)
		{
			shootControl.ShootEstimate.stir_real_angle = 0;
			shootControl.ShootEstimate.stir_real_angle_d = 0;
			shootControl.ShootEstimate.stir_angle_last = 0;
			shootControl.ShootEstimate.stir_angle_cur = 0;
			shootControl.ShootEstimate.stir_real_angle_rad = 0;
			shootControl.ShootEstimate.stir_reset_flag++;//生成重置信号
		}
		last_count = cur_count;
		last_last_count = last_count;
	}
//		//获取拨盘转动总角度
	GetStirRealAngle();
	if(!shootControl.ShootEstimate.stir_reset_flag && _stirMotorRec->frame_counter)
	{
		GetStirRealAngle();//
	}
	if(shootControl.ShootEstimate.stir_reset_flag && ext_game_robot_status.power_management_shooter_output && cur_count != last_count)
	{/*如果有重置flag，并且*/
		shootControl.ShootEstimate.stir_reset_flag = 0;
		GetStirRealAngle();
		StirTargetAngleSet();
	}
}
static void GetStirRealAngle(void)
{
	shootControl.ShootEstimate.stir_angle_cur = _stirMotorRec->pos_d;
	shootControl.ShootEstimate.stir_real_angle_d = shootControl.ShootEstimate.stir_angle_cur - shootControl.ShootEstimate.stir_angle_last;
	
//	if(shootControl.ShootEstimate.stir_real_angle_d > 720)
//		shootControl.ShootEstimate.stir_real_angle_d -= 1440.0f;
//	if(shootControl.ShootEstimate.stir_real_angle_d < -720)
//		shootControl.ShootEstimate.stir_real_angle_d += 1440.0f;	
	shootControl.ShootEstimate.stir_real_angle += shootControl.ShootEstimate.stir_real_angle_d;
	/*圈数记录*/
//	if(shootControl.ShootEstimate.stir_angle_cur<-160 && shootControl.ShootEstimate.stir_angle_last>+160)
//		shootControl.ShootEstimate.quan_shu_r++;
//	if(shootControl.ShootEstimate.stir_angle_cur>+160 && shootControl.ShootEstimate.stir_angle_last<-160)
//		shootControl.ShootEstimate.quan_shu_r--;
	/*圈数记录：先更新quan_shu_r，再计算含圈数的stir_all_angle_d，确保堵转恢复到达判断在同一坐标系 */
	if(shootControl.ShootEstimate.stir_angle_cur<-DM_MOTO_MAX_ENCODE_D	+100 && shootControl.ShootEstimate.stir_angle_last>DM_MOTO_MAX_ENCODE_D	-100)
		shootControl.ShootEstimate.quan_shu_r++;
	if(shootControl.ShootEstimate.stir_angle_cur>+DM_MOTO_MAX_ENCODE_D 	-100 && shootControl.ShootEstimate.stir_angle_last<-DM_MOTO_MAX_ENCODE_D +100)
		shootControl.ShootEstimate.quan_shu_r--;
	/*真实角度记录：包含圈数，与stir_all_target_pos_d在同一坐标系 */
	shootControl.ShootEstimate.stir_all_angle_d=(_stirMotorRec->pos_d) + (shootControl.ShootEstimate.quan_shu_r * DM_MOTO_MAX_ENCODE_D);
	
	
	shootControl.ShootEstimate.stir_angle_last = shootControl.ShootEstimate.stir_angle_cur;
	shootControl.ShootEstimate.stir_real_angle_rad = shootControl.ShootEstimate.stir_real_angle / 180.0f * 3.14f;
}
float yes_60_angle_d,temp_angle_d;
float compale_angle_d;
float dealt_d;
float temp_select_angle_d=50;//正数，如果是30就是找最近点,调小就是更容易往前
#include "arm_math.h"
/**
 * @brief 将角度对齐到最近的六等分点（带偏移）
 * @param current_angle 输入角度（单位：度）
 * @param offset 整体偏移量
 * @return 对齐后的角度（0.0f ~ 360.0f）
 */
/*2025.6.29:注意，有出现，退保护第一次反向堵转的情况，这是因为最近的六等分点在后方，被一颗大胆玩卡到了，所以确实算是堵转；这种情况应该往前转
解决方案：不一定是最近的六等分点，这个和预置位有关系*/
static void StirTargetAngleSet(void)
{
		temp_angle_d = _stirMotorRec->pos_d;
		temp_angle_d -= shootControl.ShootMotorControl.stir_preset_angle;//预置位
		dealt_d = fmod(temp_angle_d,60);
		/*对fmod的特性进行处理，若a<b，fmod会返回小的值*/
	/*你希望它尽量往前*/

		if(dealt_d>temp_select_angle_d)
			dealt_d = (dealt_d-60);
		if(dealt_d<-(60-temp_select_angle_d))
			dealt_d = (dealt_d+60);
		/*角度核心*/
		shootControl.ShootTargetInput.stir_all_target_pos_d=(_stirMotorRec->pos_d-dealt_d)+(shootControl.ShootEstimate.quan_shu_r*DM_MOTO_MAX_ENCODE_D);
		shootControl.ShootTargetInput.stir_all_target_pos_rad = shootControl.ShootTargetInput.stir_all_target_pos_d/180.0f * PI;
}
/**
 * @brief 底盘闭环控制
 */

static void ChassisControlUpdate(void)
{
	
	//实际需要的底盘速度为1.0*target + ratio*(target-real)，即引入反馈量，在原速度向量的基础上叠加反馈修正速度使得实际速度向量更快收敛到原目标速度
	chassisControl.ChassisRealNeedInput.speed_x_mps = chassisControl.ChassisCoordinateInput.speed_x_mps;
	
	chassisControl.ChassisRealNeedInput.speed_y_mps = chassisControl.ChassisCoordinateInput.speed_y_mps;
	
	chassisControl.ChassisRealNeedInput.speed_w_rps = chassisControl.ChassisCoordinateInput.speed_w_rps;
	if (_robotState->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB )
	{
		if (fabs(_normRemoteCmd->RelativeCH.ch1) > 0.1f)
		{
			chassisControl.ChassisRealNeedInput.speed_y_mps = chassisControl.ChassisCoordinateInput.speed_y_mps;
		}
		else
		chassisControl.ChassisRealNeedInput.speed_y_mps = 0.0f;
		chassisControl.ChassisRealNeedInput.speed_x_mps = 0.0f;
		chassisControl.ChassisRealNeedInput.speed_w_rps = 0.0f;
	}
	/*底盘目标速度映射到轮电机*/
	chassisControl.WheelMotorControl.target_speed_mps[LF] = + chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;
	
	chassisControl.WheelMotorControl.target_speed_mps[RF] = - chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;
	
	chassisControl.WheelMotorControl.target_speed_mps[LB] = + chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															- chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;
	
	chassisControl.WheelMotorControl.target_speed_mps[RB] = - chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															- chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;	
	
	
	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		chassisControl.WheelMotorControl.target_speed_mps[i] *= -1;
	}

	
	
	/*底盘功率控制*/
	float chassis_permitted_power = 0;
	float chassis_prior_power = 0;	
	//先验计算当前底盘所需功率值，以及当前所允许的最大输出功率值
	chassis_prior_power = ChassisPriorPowerCalc(chassisControl.WheelMotorControl.target_speed_mps);
	chassis_permitted_power = ext_game_robot_status.chassis_power_limit;
	chassisControl.ChassisRealNeedInput.power_limit_scale = 1.0;

	//计算电容最大补偿功率
	
	//老电容控制板
	#ifdef OLD_CAPACITY
	if(_superCapacity->cap_volt >= 18)
		chassisControl.SuperCapacity.max_compensate_power = 230;
	else
		chassisControl.SuperCapacity.max_compensate_power =  -0.003 * pow(_superCapacity->cap_volt,3) + 0.35 * pow(_superCapacity->cap_volt,2) + 80;
	
	#else
	if(_superCapacity->cap_volt>= 15) 
		chassisControl.SuperCapacity.max_compensate_power = 200;
	else if(_superCapacity->cap_volt > 10 &&_superCapacity->cap_volt < 15)
		chassisControl.SuperCapacity.max_compensate_power = 40 * (_superCapacity->cap_volt - 10);
	else
		chassisControl.SuperCapacity.max_compensate_power = 0;
	#endif
	
	/*起步0.6s加速*/
	static uint16_t speedCount = 0;
	speedCount++;
	if(!_normRemoteCmd->PCKeyBoard.level_key_A && !_normRemoteCmd->PCKeyBoard.level_key_D\
	 &&!_normRemoteCmd->PCKeyBoard.level_key_W && !_normRemoteCmd->PCKeyBoard.level_key_S\
	 && _robotState->chassis_mode != CHASSIS_REVOLVE)
		speedCount = 0;
	


	/*陀螺切跟随限功率计时器--切回跟随时给出瞬间功率补偿*/
	static uint16_t timeCount = 0, last_state = 0;		
	if(_robotState->chassis_mode != CHASSIS_REVOLVE && last_state == CHASSIS_REVOLVE)
		timeCount = 200;
	if(timeCount > 0)
		timeCount--;
	last_state = _robotState->chassis_mode;		
	
	/*电容电压高于一定值时放宽底盘功率限制*/
	if(_superCapacity->cap_volt >= 10.4 )	
	{	
		if(_normRemoteCmd->PCKeyBoard.level_key_SHIFT)		//shift加速（对陀螺加速在设置陀螺转速处）
			chassis_permitted_power += -0.003 * pow(_superCapacity->cap_volt,3) + 0.35 * pow(_superCapacity->cap_volt,2);
		else if(speedCount < 200)							//起步加速
			chassis_permitted_power *= 1.5;
		else if(_superCapacity->cap_volt >= 17.4)//平时状态下功率放宽
		{
			if(chassis_permitted_power <= 60)
				chassis_permitted_power *= 1.95;
			else if(chassis_permitted_power == 70)
				chassis_permitted_power *= 1.75;
			else if(chassis_permitted_power <= 90)
				chassis_permitted_power *= 1.65;
			else
				chassis_permitted_power *= 1.55;
		}
		else
			chassis_permitted_power *= 1.25;
		if(timeCount)
			chassis_permitted_power *= 0.7;					//陀螺切跟随限功率
	}
	else if(timeCount)
		chassis_permitted_power *= 0.6;
	
	if(_robotState->joint_mode == ROBOT_JOINT_MODE_CLIMB)
	   {/* 上坡模式(CLIMB)：电容加压 */
		if(_robotState->capacity_mode != NO_CAPACITY && _superCapacity->cap_volt >= 10.4)
		    chassis_permitted_power *= 2.2f;
        else   
		    chassis_permitted_power *= 2.2f;
	   }
	else if(_robotState->stand_mode == ROBOT_STAND_MODE_PRE_STAIR ||
	        _robotState->stand_mode == ROBOT_STAND_MODE_STAIR_UP)
	   {//上坡模式强制
		if(_robotState->capacity_mode != NO_CAPACITY&&_superCapacity->cap_volt >= 10.4 )
		    chassis_permitted_power = 50.0*(2.1f);
        else   
		    chassis_permitted_power = 50.0f*1.15f;
	   }
	else if(_robotState->capacity_mode == NO_CAPACITY)
		chassis_permitted_power = ext_game_robot_status.chassis_power_limit;
	

//chassis_permitted_power -= 5;//一点功率限制小shit
	chassis_permitted_power = DoubleEdgeLimiter(chassis_permitted_power, 0, ext_game_robot_status.chassis_power_limit + chassisControl.SuperCapacity.max_compensate_power);
	
	if(chassis_prior_power > chassis_permitted_power)
		chassisControl.ChassisRealNeedInput.power_limit_scale = chassis_permitted_power / chassis_prior_power ;
	
	//前馈+PD控制底盘电机闭环
	static float wheel_speed_lpf[CHASSIS_MOTOR_NUM] = {0};
	const float wheel_speed_lpf_alpha = 0.3f;  // 一阶低通系数, 截止频率 ≈ 50Hz @ 1kHz (可调 0.15~0.5)
	
	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		/* 速度反馈低通滤波: 抑制CAN回传量化噪声, 防止D项放大高频抖动 */
		wheel_speed_lpf[i] = wheel_speed_lpf_alpha * _chassisMotorRec[i].mechanical_speed_rpm
		                   + (1.0f - wheel_speed_lpf_alpha) * wheel_speed_lpf[i];
		
		chassisControl.WheelMotorControl.target_motor_output[i] = PIDUpdate(&(chassisControl.WheelMotorControl.speed_control_pid[i]),\
																			 (chassisControl.WheelMotorControl.target_speed_mps[i] * WHEEL_MPS_TO_WHEEL_RPM\
																			 - wheel_speed_lpf[i]));
		chassisControl.WheelMotorControl.target_motor_output[i] = AbsLimiter(chassisControl.WheelMotorControl.target_motor_output[i] + \
																			chassisControl.WheelMotorControl.target_speed_mps[i] * CHASSIS_MOTOR_FRONTFEED_RATIO, 16000
		)
																					*chassisControl.ChassisRealNeedInput.power_limit_scale;		//功率控制，可能要去掉;去牛魔 原来是16000注意
		//各种情况的最终手段：如果缓存能量非常少了暴力限制
		if(ext_power_heat_data.buffer_energy<10)
			chassisControl.WheelMotorControl.target_motor_output[i]*=0.5;
		
		
	}
	if (_robotState->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB )
	{
		float total_output = abs(chassisControl.WheelMotorControl.target_motor_output[LF]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[RF]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[LB]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[RB]);
		float rear_output = abs(chassisControl.WheelMotorControl.target_motor_output[LB]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[RB]);
		float scale = (rear_output > 1e-3f) ? (total_output / rear_output) : 0.0f;

		chassisControl.WheelMotorControl.target_motor_output[LF] = 0;
		chassisControl.WheelMotorControl.target_motor_output[RF] = 0;
		chassisControl.WheelMotorControl.target_motor_output[LB] = AbsLimiter(
			chassisControl.WheelMotorControl.target_motor_output[LB] * scale* 0.8, 16000);
		chassisControl.WheelMotorControl.target_motor_output[RB] = AbsLimiter(
			chassisControl.WheelMotorControl.target_motor_output[RB] * scale*0.8, 16000);
	}
	/* 上坡模式(CLIMB)：前腿功率1:3分配到后腿 */
	if (_robotState->joint_mode == ROBOT_JOINT_MODE_CLIMB)
	{
		float front_sum = fabsf(chassisControl.WheelMotorControl.target_motor_output[LF]) +
		                  fabsf(chassisControl.WheelMotorControl.target_motor_output[RF]);
		float rear_sum  = fabsf(chassisControl.WheelMotorControl.target_motor_output[LB]) +
		                  fabsf(chassisControl.WheelMotorControl.target_motor_output[RB]);
		float total = front_sum + rear_sum;

		if (total > 1e-3f)
		{
			float front_scale = (front_sum > 1e-3f) ? (total * 0.25f / front_sum) : 0.0f;
			float rear_scale  = (rear_sum  > 1e-3f) ? (total * 0.75f / rear_sum)  : 0.0f;

			chassisControl.WheelMotorControl.target_motor_output[LF] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[LF] * front_scale, 16000);
			chassisControl.WheelMotorControl.target_motor_output[RF] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[RF] * front_scale, 16000);
			chassisControl.WheelMotorControl.target_motor_output[LB] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[LB] * rear_scale, 16000);
			chassisControl.WheelMotorControl.target_motor_output[RB] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[RB] * rear_scale, 16000);
		}
	}
	static uint16_t slope_count = 0;
	static uint8_t last_slope = 0, cur_slope = 0;
	if(slope_count)
	{
		slope_count--;
		chassisControl.WheelMotorControl.target_motor_output[LF] = chassisControl.WheelMotorControl.target_motor_output[RF] = 0;
	}
	cur_slope = _robotState->auto_slope;
	if(CONTROL_FROM_REMOTE == _robotState->ctrl_terminal && cur_slope == 0 && cur_slope != last_slope)
		slope_count = 200;
	last_slope = cur_slope;
	
	#if defined CHASSIS_OFF
		for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
			chassisControl.WheelMotorControl.target_motor_output[i] = 0;
	#endif
	
	if(CONTROL_STOP == _robotState->ctrl_terminal)// || ext_game_robot_status.power_management_chassis_output == 0)
	{
		for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
			chassisControl.WheelMotorControl.target_motor_output[i] = 0;	
	}
}
static void JointControlUpdate(void)
{
	JointBodyState body_state = g_joint_body_state_obs;
	JointBodyState body_state_ctrl = {0};
	JointBodyTarget body_target = g_joint_body_target_cmd;
	static float preclimb_motor_target_angle_rad[JOINT_CTRL_MOTOR_NUM] = {
		-2.05f, -2.05f, 1.05f, 1.05f,
	};//-2.25f, -2.25f, 1.05f, 1.05f
	static uint8_t last_joint_mode = ROBOT_JOINT_MODE_NORMAL;
	static float climb_pitch_hold_d = 0.0f;
	static uint8_t prev_stand_stair = 0;  /* STAIR_UP 斜坡持续标记，函数级 static */
	extern float g_joint_motor_torque_cmd_nm[JOINT_CTRL_MOTOR_NUM];
	float mit_pos_cmd_rad[JOINT_CTRL_MOTOR_NUM] = {0};
	const float body_height_m = jointControl.JointEstimate.body_height_m;
	const float body_height_vel_mps = jointControl.JointEstimate.body_height_vel_mps;

	/*
	 * 若高度相关目标暂未填写，默认跟随当前观测，避免一上电就产生大幅 heave 力。
	 * 一旦你在 JointInputUpdate 中写入非零目标，这里会被覆盖。
	 */
	if (g_joint_body_target_cmd.heave_m == 0.0f)
		body_target.heave_m = body_height_m;
	if (g_joint_body_target_cmd.heave_vel_mps == 0.0f)
		body_target.heave_vel_mps = body_height_vel_mps;
	JointForceControlConvertBodyState(&body_state, &body_state_ctrl);
	g_joint_body_pitch_ctrl_d = body_state_ctrl.pitch_d;

	if (_robotState->joint_mode == ROBOT_JOINT_MODE_CLIMB)
	{
		/* CLIMB上坡模式走默认力控 */
		JointForceControlSetJointMode(JOINT_NORMAL, 0.0f);
	}
	else if (_robotState->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB)
	{
		if (last_joint_mode != ROBOT_JOINT_MODE_OUTCLIMB)
			climb_pitch_hold_d = 0.0f;
		JointForceControlSetJointMode(JOINT_CLIMB, 0.0f);
	}
	else
	{
		JointForceControlSetJointMode(JOINT_NORMAL, 0.0f);
	}
	JointForceControlSetStandMode(_robotState->stand_mode);
	JointForceControlSetJumpMode(_robotState->jump_mode);
	last_joint_mode = _robotState->joint_mode;

	JointForceControlStep(&body_state_ctrl,
				      &body_target,
				      body_height_m,
				      body_height_vel_mps);

	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		mit_pos_cmd_rad[i] = jointControl.JointEstimate.motor_angles_rad[i];
	}
	JointBuildFeedbackPosRadFromMotorAngles(mit_pos_cmd_rad, mit_pos_cmd_rad);
	for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
	{
		jointControl.JointMotorControl.mit_p[i] =0.0f;
		jointControl.JointMotorControl.mit_v[i] = 0.0f;
		jointControl.JointMotorControl.mit_Kp[i] = 0.0f;
		jointControl.JointMotorControl.mit_Kd[i] = 0.0f;
	}

	enum
	{
		MIT_MODE_CLIMB = 0,
		MIT_MODE_OUTCLIMB,
		MIT_MODE_STAIR_UP,
		MIT_MODE_PRECLIMB,
		MIT_MODE_REVOLVE,
		MIT_MODE_FORCE_LIMIT,
		MIT_MODE_COUNT
	};//- + - +
	static const float mit_kp_table[MIT_MODE_COUNT][JOINT_CTRL_MOTOR_NUM] = {
		[MIT_MODE_CLIMB] = { [LEG_LF] = 30.0f, [LEG_RF] = 30.0f, [LEG_RB] = 20.0f, [LEG_LB] = 20.0f },
		[MIT_MODE_OUTCLIMB] = { [LEG_LF] = 40.0f, [LEG_RF] = 40.0f, [LEG_RB] = 40.0f, [LEG_LB] = 40.0f },
		[MIT_MODE_STAIR_UP] = { [LEG_LF] = 40.0f, [LEG_RF] = 40.0f, [LEG_RB] = 40.0f, [LEG_LB] = 40.0f },
		[MIT_MODE_PRECLIMB] = { [LEG_LF] = 60.0f, [LEG_RF] = 60.0f, [LEG_RB] = 60.0f, [LEG_LB] = 60.0f },
		[MIT_MODE_REVOLVE]  = { [LEG_LF] = 30.0f, [LEG_RF] = 30.0f, [LEG_RB] = 30.0f, [LEG_LB] = 30.0f }, // TODO: 陀螺模式位控Kp
		[MIT_MODE_FORCE_LIMIT] = { [LEG_LF] = 30.0f, [LEG_RF] = 30.0f, [LEG_RB] = 30.0f, [LEG_LB] = 30.0f },
	};
	static const float mit_kd_table[MIT_MODE_COUNT][JOINT_CTRL_MOTOR_NUM] = {
		[MIT_MODE_CLIMB] = { [LEG_LF] = 1.0f, [LEG_RF] = 1.0f, [LEG_RB] = 1.0f, [LEG_LB] = 1.0f },
		[MIT_MODE_OUTCLIMB] = { [LEG_LF] = 3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 3.0f, [LEG_LB] = 3.0f },
		[MIT_MODE_STAIR_UP] = { [LEG_LF] = 3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 3.0f, [LEG_LB] = 3.0f },
		[MIT_MODE_PRECLIMB] = { [LEG_LF] = 3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 3.0f, [LEG_LB] = 3.0f },
		[MIT_MODE_REVOLVE]  = { [LEG_LF] = 3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 3.0f, [LEG_LB] = 3.0f }, // TODO: 陀螺模式位控Kd
		[MIT_MODE_FORCE_LIMIT] = { [LEG_LF] = 5.0f, [LEG_RF] = 5.0f, [LEG_RB] = 5.0f, [LEG_LB] = 5.0f },
	};
	static const float mit_tff_table[MIT_MODE_COUNT][JOINT_CTRL_MOTOR_NUM] = {
		[MIT_MODE_CLIMB] = { [LEG_LF] = -3.0f, [LEG_RF] = 3.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
		[MIT_MODE_OUTCLIMB] = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
		[MIT_MODE_STAIR_UP] = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
		[MIT_MODE_PRECLIMB] = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
		[MIT_MODE_REVOLVE]  = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f }, // TODO: 陀螺模式前馈Tff
		[MIT_MODE_FORCE_LIMIT] = { [LEG_LF] = 0.0f, [LEG_RF] = 0.0f, [LEG_RB] = 0.0f, [LEG_LB] = 0.0f },
	};
	/* ===== 统一MIT位控角度计算 ===== */
	uint8_t  mit_angle_active = 0u;
	uint8_t  mit_mode_idx      = MIT_MODE_FORCE_LIMIT;
	float    motor_target_angle_rad[JOINT_CTRL_MOTOR_NUM] = {0};

	if (_robotState->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB)
	{
		mit_mode_idx = MIT_MODE_OUTCLIMB;
		motor_target_angle_rad[LEG_LF] = -2.65f;
		motor_target_angle_rad[LEG_RF] = -2.65f;
		motor_target_angle_rad[LEG_RB] = -0.35f;
		motor_target_angle_rad[LEG_LB] = -0.35f;
		mit_angle_active = 1u;
	}
	else if (_robotState->stand_mode == ROBOT_STAND_MODE_PRE_STAIR)
	{
		/* PRE_STAIR: 前腿MIT位控收腿到-2.50rad，后腿继续力控 */
		float front_angles_rad[JOINT_CTRL_MOTOR_NUM];
		float front_pos_rad[JOINT_CTRL_MOTOR_NUM];
		front_angles_rad[LEG_LF] = -2.60f;
		front_angles_rad[LEG_RF] = -2.60f;
		front_angles_rad[LEG_RB] = jointControl.JointEstimate.motor_angles_rad[LEG_RB];
		front_angles_rad[LEG_LB] = jointControl.JointEstimate.motor_angles_rad[LEG_LB];
		JointBuildFeedbackPosRadFromMotorAngles(front_angles_rad, front_pos_rad);
		/* 仅前腿走MIT位控，后腿保持力控(Tff由JointForceControlStep写入) */
		for (uint8_t leg = LEG_LF; leg <= LEG_RF; leg++)
		{
			jointControl.JointMotorControl.mit_p[leg]  = front_pos_rad[leg];
			jointControl.JointMotorControl.mit_v[leg]  = 0.0f;
			jointControl.JointMotorControl.mit_Kp[leg] = 15.0f;
			jointControl.JointMotorControl.mit_Kd[leg] = 0.0f;
		}
		/* 不设 mit_angle_active，避免统一应用覆盖后腿力控Tff */
	}
	else if (_robotState->stand_mode == ROBOT_STAND_MODE_STAIR_UP)
	{
		mit_mode_idx = MIT_MODE_STAIR_UP;
		/* 目标角度缓慢变化，每周期收敛0.1rad */
		static float    ramp_angle[JOINT_CTRL_MOTOR_NUM] = {0};
		static uint8_t  stair_up_ramp_inited = 0;
		const float target_angle[JOINT_CTRL_MOTOR_NUM] = {
			[LEG_LF] = -2.60f, [LEG_RF] = -2.60f,
			[LEG_RB] =  1.70f, [LEG_LB] =  1.70f,
		};
		/* 首次进入或从其他模式切回时，重新从当前角度开始斜坡 */
		if (!stair_up_ramp_inited || !prev_stand_stair)
		{
			for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
				ramp_angle[i] = jointControl.JointEstimate.motor_angles_rad[i];
			stair_up_ramp_inited = 1u;
		}
		for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
		{
			float err = target_angle[i] - ramp_angle[i];
			if (err >  0.1f)       ramp_angle[i] += 0.003f;
			else if (err < -0.1f)  ramp_angle[i] -= 0.003f;
			else                   ramp_angle[i] = target_angle[i];
			motor_target_angle_rad[i] = ramp_angle[i];
		}
		mit_angle_active = 1u;
		prev_stand_stair = 1u;
	}
	else if (_robotState->joint_mode == ROBOT_JOINT_MODE_PRECLIMB)
	{
		mit_mode_idx = MIT_MODE_PRECLIMB;
		const float pitch_min_d = 0.0f;
		const float pitch_max_d = 20.0f;
		const float front_angle_pitch0_rad  = -2.25f;
		const float front_angle_pitch20_rad = -2.66f;
		float pitch_d = body_state_ctrl.pitch_d;
		const float front_slope = (front_angle_pitch20_rad - front_angle_pitch0_rad) / (pitch_max_d - pitch_min_d);

		if (pitch_d < pitch_min_d)
			pitch_d = pitch_min_d;
		else if (pitch_d > pitch_max_d)
			pitch_d = pitch_max_d;

		const float front_target_angle_rad = front_angle_pitch0_rad + front_slope * (pitch_d - pitch_min_d);
		motor_target_angle_rad[LEG_LF] = front_target_angle_rad;
		motor_target_angle_rad[LEG_RF] = front_target_angle_rad;
		motor_target_angle_rad[LEG_RB] = preclimb_motor_target_angle_rad[LEG_RB];
		motor_target_angle_rad[LEG_LB] = preclimb_motor_target_angle_rad[LEG_LB];
		mit_angle_active = 1u;
	}

	/* 退出 STAIR_UP 时清除持续标记，使下次重新斜坡 */
	if (_robotState->stand_mode != ROBOT_STAND_MODE_STAIR_UP)
		prev_stand_stair = 0u;

	uint16_t rev_exit_hold_cnt = 0;
	/* 陀螺退出保持：退出陀螺后强制位控200周期，平滑过渡 */
	{
		static float    rev_exit_hold_angle_rad[JOINT_CTRL_MOTOR_NUM] = {0};
		static uint8_t  last_revolve = 0;

		if (_robotState->chassis_mode != CHASSIS_REVOLVE && last_revolve)
		{
			rev_exit_hold_angle_rad[LEG_LF] = -1.702f;
			rev_exit_hold_angle_rad[LEG_RF] = -1.702f;
			rev_exit_hold_angle_rad[LEG_RB] =  1.319f;
			rev_exit_hold_angle_rad[LEG_LB] =  1.319f;
			rev_exit_hold_cnt = 600;
		}
		last_revolve = (_robotState->chassis_mode == CHASSIS_REVOLVE) ? 1u : 0u;

		if (rev_exit_hold_cnt > 0)
		{
			rev_exit_hold_cnt--;
			for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
				motor_target_angle_rad[i] = rev_exit_hold_angle_rad[i];
			mit_mode_idx    = MIT_MODE_REVOLVE;
			mit_angle_active = 1u;
		}
	}

	/* ===== 统一应用MIT位控参数 ===== */
	if (mit_angle_active)
	{
		JointBuildFeedbackPosRadFromMotorAngles(motor_target_angle_rad, mit_pos_cmd_rad);
		for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
		{
			jointControl.JointMotorControl.mit_p[i]   = mit_pos_cmd_rad[i];
			jointControl.JointMotorControl.mit_v[i]   = 0.0f;
			jointControl.JointMotorControl.mit_Kp[i]  = mit_kp_table[mit_mode_idx][i];
			jointControl.JointMotorControl.mit_Kd[i]  = mit_kd_table[mit_mode_idx][i];
			jointControl.JointMotorControl.mit_Tff[i] = mit_tff_table[mit_mode_idx][i];
		}
	}

	/* ===== 力控模式下强制位置限幅 ===== */
	{
		float motor_angle_min_rad[JOINT_CTRL_MOTOR_NUM] = {0};
		float motor_angle_max_rad[JOINT_CTRL_MOTOR_NUM] = {0};
		uint8_t limit_hit[JOINT_CTRL_MOTOR_NUM] = {0};
		uint8_t limit_active = 0u;

		JointGetMotorAngleLimitsRad(motor_angle_min_rad, motor_angle_max_rad);
		if(_robotState->stand_mode==ROBOT_STAND_MODE_NORMAL)
		{
			motor_angle_max_rad[LEG_LF]-=0.20f;
			motor_angle_max_rad[LEG_RF]-=0.20f;
		}
		const float limit_inset_rad = 0.05f;
		for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
		{
			const float angle = jointControl.JointEstimate.motor_angles_rad[i];
			if (angle < motor_angle_min_rad[i])
			{
				motor_target_angle_rad[i] = motor_angle_min_rad[i] + limit_inset_rad;
				limit_hit[i] = 1u;
				limit_active = 1u;
			}
			else if (angle > motor_angle_max_rad[i])
			{
				motor_target_angle_rad[i] = motor_angle_max_rad[i] - limit_inset_rad;
				limit_hit[i] = 1u;
				limit_active = 1u;
			}
			else
			{
				motor_target_angle_rad[i] = angle;
			}
		}

		if (limit_active)
		{
			JointBuildFeedbackPosRadFromMotorAngles(motor_target_angle_rad, mit_pos_cmd_rad);
			for (uint8_t i = 0; i < JOINT_CTRL_MOTOR_NUM; i++)
			{
				if (!limit_hit[i])
					continue;
				jointControl.JointMotorControl.mit_p[i]  = mit_pos_cmd_rad[i];
				jointControl.JointMotorControl.mit_v[i]  = 0.0f;
				jointControl.JointMotorControl.mit_Kp[i] = mit_kp_table[MIT_MODE_FORCE_LIMIT][i];
				jointControl.JointMotorControl.mit_Kd[i] = mit_kd_table[MIT_MODE_FORCE_LIMIT][i];
			}
		}
	}

	/* Y方向急停时前腿位控保持，防止惯性压弯前腿角度突变 */
	/* PRE_DOWN_STAIR 模式下跳过急停位控 */
	if (_robotState->stand_mode != ROBOT_STAND_MODE_PRE_DOWN_STAIR&&_robotState->stand_mode !=ROBOT_STAND_MODE_PRE_STAIR&&_robotState->stand_mode !=ROBOT_STAND_MODE_STAIR_UP&& rev_exit_hold_cnt == 0)
	{
		static float last_speed_y_mps = 0.0f;
		static uint16_t decel_hold_cnt = 0;
		static float decel_hold_angle_rad[2] = {0.0f, 0.0f}; /* LF, RF */
		const float decel_enter_speed_mps = 0.30f;  /* 高于此速度视为"运动中" */
		const float decel_exit_speed_mps = 0.20f;   /* 低于此速度视为"已急停" */
		const uint16_t decel_hold_cycles = 200;     /* 位控保持周期数 */

		/* 检测急停边沿：目标速度从高位骤降至低位 */
		if (last_speed_y_mps > decel_enter_speed_mps &&
		    chassisControl.ChassisRealNeedInput.speed_y_mps < decel_exit_speed_mps)
		{
			decel_hold_cnt = decel_hold_cycles;
			/* 记录急停瞬间的前腿关节角作为位控目标 */
			decel_hold_angle_rad[0] = jointControl.JointEstimate.motor_angles_rad[LEG_LF];
			decel_hold_angle_rad[1] = jointControl.JointEstimate.motor_angles_rad[LEG_RF];
		}

		if (decel_hold_cnt > 0)
		{
			decel_hold_cnt--;
			/* 前腿切位控模式，锁定急停瞬间角度 */
			float hold_angles_rad[JOINT_CTRL_MOTOR_NUM];
			hold_angles_rad[LEG_LF] = decel_hold_angle_rad[0];
			hold_angles_rad[LEG_RF] = decel_hold_angle_rad[1];
			hold_angles_rad[LEG_RB] = jointControl.JointEstimate.motor_angles_rad[LEG_RB];
			hold_angles_rad[LEG_LB] = jointControl.JointEstimate.motor_angles_rad[LEG_LB];

			float hold_pos_rad[JOINT_CTRL_MOTOR_NUM];
			JointBuildFeedbackPosRadFromMotorAngles(hold_angles_rad, hold_pos_rad);

			for (uint8_t leg = LEG_LF; leg <= LEG_RF; leg++)
			{
				jointControl.JointMotorControl.mit_p[leg]   = hold_pos_rad[leg];
				jointControl.JointMotorControl.mit_v[leg]   = 0.0f;
				jointControl.JointMotorControl.mit_Kp[leg]  = mit_kp_table[MIT_MODE_FORCE_LIMIT][leg];
				jointControl.JointMotorControl.mit_Kd[leg]  = mit_kd_table[MIT_MODE_FORCE_LIMIT][leg];
			}
		}

		last_speed_y_mps = chassisControl.ChassisRealNeedInput.speed_y_mps;
	}

	/* X方向移动时前腿位控保持，防止横向惯性压弯前腿角度突变 */
	/* PRE_DOWN_STAIR 模式下跳过 */
	if (_robotState->stand_mode != ROBOT_STAND_MODE_PRE_DOWN_STAIR&&_robotState->chassis_mode != CHASSIS_REVOLVE&&_robotState->stand_mode !=ROBOT_STAND_MODE_PRE_STAIR&&_robotState->stand_mode !=ROBOT_STAND_MODE_STAIR_UP&& rev_exit_hold_cnt == 0)
	{
		static float last_speed_x_mps = 0.0f;
		static float x_hold_angle_rad[2] = {0.0f, 0.0f}; /* LF, RF */
		static uint8_t x_hold_active = 0;
		const float x_speed_deadband = 0.05f;  /* 低于此值视为无X输入 */

		/* 检测X方向速度输入开始边沿：从静止到运动，记录输入前的关节角 */
		if (fabs(last_speed_x_mps) < x_speed_deadband &&
		    fabs(chassisControl.ChassisRealNeedInput.speed_x_mps) >= x_speed_deadband)
		{
			x_hold_angle_rad[0] = jointControl.JointEstimate.motor_angles_rad[LEG_LF];
			x_hold_angle_rad[1] = jointControl.JointEstimate.motor_angles_rad[LEG_RF];
			x_hold_active = 1;
		}

		/* X方向速度输入结束，释放保持 */
		if (fabs(chassisControl.ChassisRealNeedInput.speed_x_mps) < x_speed_deadband)
		{
			x_hold_active = 0;
		}

		if (x_hold_active)
		{
			/* 前腿切位控模式，锁定速度输入前的角度 */
			float hold_angles_rad[JOINT_CTRL_MOTOR_NUM];
			hold_angles_rad[LEG_LF] = x_hold_angle_rad[0];
			hold_angles_rad[LEG_RF] = x_hold_angle_rad[1];
			hold_angles_rad[LEG_RB] = jointControl.JointEstimate.motor_angles_rad[LEG_RB];
			hold_angles_rad[LEG_LB] = jointControl.JointEstimate.motor_angles_rad[LEG_LB];

			float hold_pos_rad[JOINT_CTRL_MOTOR_NUM];
			JointBuildFeedbackPosRadFromMotorAngles(hold_angles_rad, hold_pos_rad);

			for (uint8_t leg = LEG_LF; leg <= LEG_RF; leg++)
			{
				jointControl.JointMotorControl.mit_p[leg]   = hold_pos_rad[leg];
				jointControl.JointMotorControl.mit_v[leg]   = 0.0f;
				jointControl.JointMotorControl.mit_Kp[leg]  = mit_kp_table[MIT_MODE_FORCE_LIMIT][leg];
				jointControl.JointMotorControl.mit_Kd[leg]  = mit_kd_table[MIT_MODE_FORCE_LIMIT][leg];
			}
		}

		last_speed_x_mps = chassisControl.ChassisRealNeedInput.speed_x_mps;
	}
}

/// @brief 底盘先验功率计算
/// @param target_motor_mps 4个轮电机的目标轮速（m/s）
/// @return float 返回值为底盘功率模型计算出的先验功率值，底盘功率模型根据m3508电机建模*4+const_cost计算
/// @note 直流电机功率模型为：P_in = τ*w/9.55 + k_1*w*w + k_2*τ*τ + const_cost 
float prior_chassis_power = 0;//底盘功率模型先验计算值
static float ChassisPriorPowerCalc(float* target_motor_mps)
{
	prior_chassis_power = 0;//底盘功率模型先验计算值
	int16_t tatget_motor_output = 0;
	float prior_motor_power = 0;

	const float toque_coefficient = 1.99688994e-6f;//功率模型的第一项系数，物理规律，固定
	const float k_1 = 4e-07;//k_1,k_2均为辨识参数，对于同一电机型号，忽略铜损等电机差异性损耗，都采用同一参数
	const float k_2 = 2.5e-07;
	const float const_cost = 0.5;//除电机外的其他损耗，如电调功率


	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		tatget_motor_output = PIDUpdatePrior(chassisControl.WheelMotorControl.speed_control_pid[i],\
											(chassisControl.WheelMotorControl.target_speed_mps[i] * WHEEL_MPS_TO_WHEEL_RPM\
											- _chassisMotorRec[i].mechanical_speed_rpm))\
							+ chassisControl.WheelMotorControl.target_speed_mps[i] * CHASSIS_MOTOR_FRONTFEED_RATIO;
		
		tatget_motor_output = AbsLimiter(tatget_motor_output, 16000);

		prior_motor_power = tatget_motor_output * _chassisMotorRec[i].mechanical_speed_rpm * toque_coefficient +\
							k_1 * square(_chassisMotorRec[i].mechanical_speed_rpm) +\
							k_2 * square(tatget_motor_output) +\
							const_cost;
		prior_chassis_power += prior_motor_power;
	}
	prior_chassis_power = AverageFilterUpdate(&PowerFilter, prior_chassis_power);
	return prior_chassis_power;
}

static void GimbalControlUpdate(void)
{
	
	#if defined GIMBAL_OFF
		gimbalControl.GimbalMotorControl.pitch_target_output = gimbalControl.GimbalMotorControl.yaw_target_output = 0;
	#endif
	
	if(CONTROL_STOP == _robotState->ctrl_terminal)
	{
		gimbalControl.GimbalMotorControl.yaw_target_output = 0;
	}	
}

/**
 * @brief 发射闭环控制
 */ 
static void ShootControlUpdate(void)    
{
	if(shootControl.ShootEstimate.stir_block_flag && stir_stall_recovery_state == 0)
		shootControl.ShootTargetInput.stir_target_vol = 0;//堵转恢复期间允许电机运动
	else
	{
		if(gimbalControl.GimbalEstimate.pitch_angle_d<0){
		shootControl.ShootTargetInput.stir_target_vol = STIR_MAX_SPEED_LOW;
		}
		else
		{
			shootControl.ShootTargetInput.stir_target_vol = STIR_MAX_SPEED;
		}
	}
	/*一下有保护*/
	if(CONTROL_STOP == _robotState->ctrl_terminal)
	{
		shootControl.ShootEstimate.stir_enableflag_desire = DISABLE; // 期望失能
	}
	
	#if defined SHOOT_OFF
		shootControl.ShootMotorControl.fric_target_output[LEFT] = shootControl.ShootMotorControl.fric_target_output[RIGHT] = 0;
		shootControl.ShootEstimate.stir_enableflag_desire = DISABLE; // 期望失能
	#endif
}

/**
 * @file    gimbalControl.c
 * @brief   云台控制模块 —— yaw轴输入决策、观测估计、闭环控制
 * @note    从 robot_control_task 拆分，集中管理云台所有控制逻辑
 */

#include "gimbalControl.h"
#include "algorism.h"
#include "pid.h"
#include "adrc.h"
#include "state_task.h"
#include "general_task_include.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"
#include "CAN_driver.h"
#include "math.h"

/*---------------------------------------------------------------------------全局实例-------------------------------------------------------------------------------------------*/
GimbalControl gimbalControl = {0};
const GimbalControl* _gimbalControl = &gimbalControl;

/*---------------------------------------------------------------------------模块级变量-----------------------------------------------------------------------------------------*/
static uint8_t c_key_aim_yaw_lock = 0;  /* C键自瞄yaw保护锁: 1=禁止GimbalControlUpdate覆盖yaw目标 */

float yaw_dm_forward_offset_rad = -2.54f; /* DM yaw编码器"云台正前方"机械角(rad) */
uint8_t shit_delay_count = 0;             /* 状态切换延时计数器 */

/* GimbalInputUpdate 专用静态变量 */
static float micro_yaw = 0;
static int   temp_yaw_count = 0;
static int   last_temp_yaww = 0;

/* yaw估计值一阶低通 */
static SmoothFilter yaw_estimate_lpf;

/*---------------------------------------------------------------------------外部引用-----------------------------------------------------------------------------------------*/
/* RS485 接收数据 */
extern volatile float gimbal_yaw_target_rx_d;
extern volatile float gimbal_yaw_dps_rx;
extern volatile float gimbal_yaw_rx_d;
extern volatile float gimbal_pitch_rx_d;
extern volatile float gimbal_pitch_dps_rx;
extern volatile uint8_t gimbal_yaw_rx_valid;

/* DM电机 / 云台位姿 */
extern DMJ4310MotorRec DMyawMotorRec;
extern DJIGMotorRec yawMotorRec;
extern Pose gimbalPose;
extern RobotState robotState;

/* 发射控制 */
extern ShootControl shootControl;

/* 鼠标滤波 */
extern SmoothFilter MouseFilterX;

/*---------------------------------------------------------------------------初始化-------------------------------------------------------------------------------------------*/

/**
 * @brief 云台控制初始化（yaw轴PID / LTD / TD）
 * @note  由 ControlInit() 调用
 */
void GimbalInit(void)
{
	LTDInitialize(&gimbalControl.GimbalMotorControl.yaw_LTD, 20, 0.003, -180, 180);
	TDInitialize(&gimbalControl.GimbalMotorControl.yaw_TD, 20, 0.003, 1, -180, 180);
	/* 位置环PI, 速度环PD */
	PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_pos_pid,  4.0,  0.0, 0,     100, 40);
	PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_speed_pid, 0.09, 0.0008,    0.0, 300,   10);
	SmoothFilterInitialize(&yaw_estimate_lpf, 0.85f);
}

/*---------------------------------------------------------------------------输入决策更新-------------------------------------------------------------------------------------*/

/**
 * @brief 云台yaw输入决策更新（pitch由上板独立控制）
 */
void GimbalInputUpdate(void)
{
	switch(_robotState->ctrl_terminal)
	{
		case CONTROL_STOP:
			gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
		break;

		case CONTROL_FROM_REMOTE:
			if((_robotState->sniper==SNIPER_ON)){
				if(_robotState->world_enable == WORLD_ENABLE_ON)
				{
					gimbalControl.GimbalTargetInput.yaw_angle_d = gimbal_yaw_target_rx_d;
				}
				else
				{
					gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 0.1 - _normRemoteCmd->RelativeCH.ch2 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
				}
			}
			if(_robotState->sniper==SNIPER_OFF){
				gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 1.0 - _normRemoteCmd->RelativeCH.ch2 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
			}
			#ifndef SHOOT_OFF
			shootControl.ShootEstimate.stir_enableflag_desire = ENABLE;
		#else
			shootControl.ShootEstimate.stir_enableflag_desire = DISABLE;
		#endif

		break;

		case CONTROL_FROM_PC:
		{
			static uint8_t last_aim_mode_state = 0;
			if(_robotState->aim_mode && _robotState->sniper == SNIPER_ON)
			{
				gimbalControl.GimbalTargetInput.yaw_angle_d = gimbal_yaw_target_rx_d;
				c_key_aim_yaw_lock = 1;
				robotState.aim_mode = 0;
				last_aim_mode_state = 1;
			}
			else
			{
				uint8_t just_released_c = last_aim_mode_state;
				last_aim_mode_state = 0;
				float temp_yaw = 0;
				float smooth = 0.02f;
				if(_robotState->sniper==SNIPER_ON){
					smooth = 0.015f;

					/* AD键yaw微调 */
					int temp_yaw_an = (_normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A);
					if (temp_yaw_an != 0){
						last_temp_yaww = temp_yaw_an;
						temp_yaw_count++;
						if(temp_yaw_count > 10){
							temp_yaw = (_normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A) * 0.01f;
							micro_yaw += (_normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A) * 0.01f;
						}
					}
					if (temp_yaw_an == 0 && last_temp_yaww != 0){
						if(temp_yaw_count <= 10){
							micro_yaw += last_temp_yaww * 0.1f;
							temp_yaw += last_temp_yaww * 0.1f;
						}
						last_temp_yaww = 0;
						temp_yaw_count = 0;
					}
				}

				if(_robotState->follow == FOLLOW_OFF){
					if(!(_robotState->sniper == SNIPER_ON && _robotState->mouse_fix == MOUSE_FIX_ON)){
						temp_yaw += SmoothFilterUpdate(&MouseFilterX, _normRemoteCmd->PCMouse.mouse_speed_x) * smooth;
					}
					micro_yaw = 0;
				}

				if(_robotState->follow != FOLLOW_ON)
				{
					if (!gimbal_yaw_rx_valid && _robotState->sniper == SNIPER_OFF)
					{
						gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
					}
					else
					{
						gimbalControl.GimbalTargetInput.yaw_angle_d += temp_yaw + micro_yaw;
					}
				}
			}

			if(c_key_aim_yaw_lock && fabs(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d, -180.0f, 180.0f)) < 3.0f)
				c_key_aim_yaw_lock = 0;

			#ifndef GIMBAL_OFF
				shootControl.ShootEstimate.stir_enableflag_desire = ENABLE;
			#else
				shootControl.ShootEstimate.stir_enableflag_desire = DISABLE;
			#endif
		}
		break;
	}

	/* yaw角限幅 */
	gimbalControl.GimbalTargetInput.yaw_angle_d = AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d, -180, 180);
}

/*---------------------------------------------------------------------------观测更新-----------------------------------------------------------------------------------------*/

/**
 * @brief 云台相关观测数据更新
 */
void GimbalEstimateUpdate(void)
{
	/* 当前由 GimbalPoseUpdate() 在外层IMU任务中更新，此处预留 */
}

/**
 * @brief 云台位姿观测更新（由IMU任务调用，周期与IMU一致）
 * @param pitch_angle   观测pitch轴角，单位degree
 * @param pitch_angle_w 观测pitch角速度，单位rad/s
 * @param yaw_angle     观测yaw轴角，单位degree
 * @param yaw_angle_w   观测yaw角速度，单位rad/s
 * @param roll_angle    观测roll轴角，单位degree
 * @param roll_angle_w  观测roll角速度，单位rad/s
 */
void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w,
                      float roll_angle, float roll_angle_w)
{
	gimbalControl.GimbalEstimate.roll_angle_d = roll_angle;
	gimbalControl.GimbalEstimate.roll_angular_velocity_dps = roll_angle_w;

	/* yaw编码器角度(DM电机): pos_d(rad) -> 减offset -> 转degree */
	float yaw_enc_deg = AngleLimit((DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f, -180.0f, 180.0f);

	if (_robotState->sniper == SNIPER_OFF)
	{
		gimbalControl.GimbalEstimate.yaw_angle_d = gimbal_yaw_rx_d;
		gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = gimbal_yaw_dps_rx;
	}
	else
	{
		gimbalControl.GimbalEstimate.yaw_angle_d = yaw_enc_deg;
		gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = gimbal_yaw_dps_rx;
	}

	gimbalControl.GimbalEstimate.pitch_angle_d = gimbal_pitch_rx_d;
	gimbalControl.GimbalEstimate.pitch_angular_velocity_dps = gimbal_pitch_dps_rx;

	if (shit_delay_count < 200)
		shit_delay_count++;

	if (lastRobotState.lens != _robotState->sniper)
	{
		shit_delay_count = 0;
		gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
		LTD_Reset(&gimbalControl.GimbalMotorControl.yaw_LTD, gimbalControl.GimbalEstimate.yaw_angle_d);
		lastRobotState.lens = _robotState->sniper;
	}
}

/*---------------------------------------------------------------------------闭环控制-----------------------------------------------------------------------------------------*/

/**
 * @brief 云台yaw轴闭环控制（MIT力矩/速度指令输出）
 */
void GimbalControlUpdate(void)
{

	#if defined GIMBAL_OFF
		gimbalControl.GimbalMotorControl.yaw_target_output = 0;
	#endif

	//---------------------------------------------yaw轴控制,MIT&IMU和编码器--------------------------------------------------------------------------------------------------
		gimbalControl.GimbalEstimate.yaw_angle_d = SmoothFilterUpdate(&yaw_estimate_lpf, gimbalControl.GimbalEstimate.yaw_angle_d);
		float yaw_pos_err_d = AngleLimit(gimbalControl.GimbalMotorControl.yaw_LTD.x1 - gimbalControl.GimbalEstimate.yaw_angle_d, -180.0f, 180.0f);
		if(fabs(yaw_pos_err_d) < 0.4f){
			// PIDSetParam(&gimbalControl.GimbalMotorControl.yaw_pos_pid,   3.0f, 0.0f, 0.0f,   100.0f, 40.0f);
			// PIDSetParam(&gimbalControl.GimbalMotorControl.yaw_speed_pid, 0.12f, 0.0f, 0.010f, 0.0f,   10.0f);
		}
		else{
			//PIDSetParam(&gimbalControl.GimbalMotorControl.yaw_pos_pid,   4.0f, 0.0f, 0.0f,   100.0f, 40.0f);
		}
		LTDUpdate(&gimbalControl.GimbalMotorControl.yaw_LTD, gimbalControl.GimbalTargetInput.yaw_angle_d);
		
		PIDUpdate(&gimbalControl.GimbalMotorControl.yaw_pos_pid, yaw_pos_err_d);
		gimbalControl.GimbalMotorControl.w_d =  gimbalControl.GimbalMotorControl.yaw_LTD.x2+gimbalControl.GimbalMotorControl.yaw_pos_pid.output;
		gimbalControl.GimbalTargetInput.yaw_angular_velocity_dps = gimbalControl.GimbalMotorControl.w_d;
		PIDUpdate(&gimbalControl.GimbalMotorControl.yaw_speed_pid, gimbalControl.GimbalMotorControl.w_d - gimbalControl.GimbalEstimate.yaw_angular_velocity_dps);
		gimbalControl.GimbalMotorControl.yaw_target_output=-(gimbalControl.GimbalMotorControl.yaw_speed_pid.output );
		//力矩方向:逆时针是正,顺时针负.大概零点几
		//x1:初始位置顺时针是正
		//x2(速度方向):逆时针是负,顺时针是正,大概十几,几十
		//假设向顺时针方向旋转,pos_error是正,pos_output是正值对的,speed_output也是正,实际应该给负值
	if(CONTROL_STOP != _robotState->ctrl_terminal){
		/* ===== yaw轴 MIT 力矩/速度指令输出 ===== */
		if(_robotState->sniper == SNIPER_ON){
			MIT_SetParam(&gimbalControl.GimbalMotorControl.mit,
				0.0f, 0.0f,
				0.0f, 0.0f,
				AbsLimiter(gimbalControl.GimbalMotorControl.yaw_target_output, 10.0f));
		}
		else
		{
			/* 普通模式 */
			if (!gimbal_yaw_rx_valid){
				/* RS485丢失 → 开环MIT速度控制，鼠标映射yaw转速 */
				MIT_SetParam(&gimbalControl.GimbalMotorControl.mit,
					DMyawMotorRec.pos_d,(float)_normRemoteCmd->PCMouse.mouse_speed_x * 0.03f,
					0.5f, 0.3f, 0.0f);
			}
			else{
				/* RS485正常 → 双环PID输出转力矩 */
				MIT_SetParam(&gimbalControl.GimbalMotorControl.mit,
					0.0f, 0.0f, 0.0f, 0.0f,
					AbsLimiter(gimbalControl.GimbalMotorControl.yaw_target_output, 10.0f));
			}
		}
	}
	else{
		/* CONTROL_STOP: 停转保持当前位置 */
		MIT_SetParam(&gimbalControl.GimbalMotorControl.mit, DMyawMotorRec.pos_d, 0.0f, 0.0f, 0.0f, 0.0f);
	}
//------------------------------------------------------------------------------------------------------------------------------------------------
	/*共同保护*/
	static uint8_t last_ctrl_terminal = CONTROL_STOP;
	if(last_ctrl_terminal == CONTROL_STOP && _robotState->ctrl_terminal != CONTROL_STOP)
	{
		/* 退保护边沿：对齐目标和跟踪器状态，避免上电瞬间冲击 */
		gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
		LTD_Reset(&gimbalControl.GimbalMotorControl.yaw_LTD, gimbalControl.GimbalEstimate.yaw_angle_d);
		MIT_SetParam(&gimbalControl.GimbalMotorControl.mit, DMyawMotorRec.pos_d, 0.0f, 0.0f, 0.0f, 0.0f);
		shit_delay_count = 0;
	}
	last_ctrl_terminal = _robotState->ctrl_terminal;

	if((CONTROL_STOP == _robotState->ctrl_terminal||shit_delay_count<30) && !c_key_aim_yaw_lock)
	{

		gimbalControl.GimbalTargetInput.yaw_angle_d=gimbalControl.GimbalEstimate.yaw_angle_d;
		gimbalControl.GimbalMotorControl.yaw_LTD.error_sum=0;
	}
}

/**
 * @file    gimbalControl.c
 * @brief   云台控制模块 —— yaw轴输入决策、观测估计、闭环控制
 * @note    从 robot_control_task 拆分，集中管理云台所有控制逻辑
 */

#include "gimbalControl.h"
#include "general_task_include.h"
#include "../../PrivateDrivers/Board2Borad/Board2Board.h"


/*---------------------------------------------------------------------------全局实例-------------------------------------------------------------------------------------------*/
GimbalControl gimbalControl = {0};
const GimbalControl* _gimbalControl = &gimbalControl;

/*---------------------------------------------------------------------------模块级变量-----------------------------------------------------------------------------------------*/

uint8_t shit_delay_count = 0;             /* 状态切换延时计数器 */

/* GimbalInputUpdate 专用静态变量 */
static float micro_yaw = 0;
static int   temp_yaw_count = 0;
static int   last_temp_yaww = 0;



/*---------------------------------------------------------------------------外部引用-----------------------------------------------------------------------------------------*/
/* RS485 接收数据 */
extern volatile float gimbal_yaw_target_rx_d;
extern volatile float gimbal_yaw_dps_rx;
extern volatile float gimbal_yaw_rx_d;
extern volatile float gimbal_pitch_rx_d;
extern volatile float gimbal_pitch_dps_rx;
extern volatile uint8_t gimbal_yaw_rx_valid;

/* DM电机 / 云台位姿 */
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

}
/*---------------------------------------------------------------------------输入决策更新-------------------------------------------------------------------------------------*/

/**
 * @brief 云台yaw输入决策更新（pitch由上板独立控制）
 */
void GimbalInputUpdate(void)
{
	switch(pDecisionAO->ctrl_terminal)
	{
		case CONTROL_STOP:
			gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
		break;

		case CONTROL_FROM_REMOTE:
			if((pDecisionAO->sniper==SNIPER_ON)){
				if(pDecisionAO->world_enable == WORLD_ENABLE_ON)
				{
					gimbalControl.GimbalTargetInput.yaw_angle_d = gimbal_yaw_target_rx_d;
				}
				else
				{
					gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 0.1 - _normRemoteCmd->RelativeCH.ch2 * (pDecisionAO->chassis_mode == CHASSIS_SEPARATE));
				}
			}
			if(pDecisionAO->sniper==SNIPER_OFF){
				gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 1.0 - _normRemoteCmd->RelativeCH.ch2 * (pDecisionAO->chassis_mode == CHASSIS_SEPARATE));
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
			if(_robotState->aim_mode && pDecisionAO->sniper == SNIPER_ON)
			{
				gimbalControl.GimbalTargetInput.yaw_angle_d = gimbal_yaw_target_rx_d;
				// robotState.aim_mode = 0;  /* TODO: aim_mode needs to be migrated to DecisionAO separately */
				last_aim_mode_state = 1;
			}
			else
			{
				uint8_t just_released_c = last_aim_mode_state;
				last_aim_mode_state = 0;
				float temp_yaw = 0;
				float smooth = 0.02f;
				if(pDecisionAO->sniper==SNIPER_ON){
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

				if(pDecisionAO->sniper == SNIPER_OFF){
					if(!(pDecisionAO->sniper == SNIPER_ON && pDecisionAO->mouse_fix == MOUSE_FIX_ON)){
						temp_yaw += SmoothFilterUpdate(&MouseFilterX, _normRemoteCmd->PCMouse.mouse_speed_x) * smooth;
					}
					micro_yaw = 0;
				}

				if(pDecisionAO->sniper != SNIPER_ON)
				{
					if (!gimbal_yaw_rx_valid && pDecisionAO->sniper == SNIPER_OFF)
					{
						gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
					}
					else
					{
						gimbalControl.GimbalTargetInput.yaw_angle_d += temp_yaw + micro_yaw;
					}
				}
			}


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

void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w,
                      float roll_angle, float roll_angle_w)
{
	gimbalControl.GimbalEstimate.roll_angle_d = roll_angle;
	gimbalControl.GimbalEstimate.roll_angular_velocity_dps = roll_angle_w;

	/* B2B 心跳 + 数据接收（yaw DM编码器已搬迁至上板，数据由B2B 0x228提供） */
	B2B_PoseAliveTick();

	/* NaN/Inf 保护：若B2B数据异常则保留上一次有效值 */
	if (isfinite(gimbal_yaw_rx_d))
		gimbalControl.GimbalEstimate.yaw_angle_d = gimbal_yaw_rx_d;
	if (isfinite(gimbal_yaw_dps_rx))
		gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = gimbal_yaw_dps_rx;
	if (isfinite(gimbal_pitch_rx_d))
		gimbalControl.GimbalEstimate.pitch_angle_d = gimbal_pitch_rx_d;
		
	if (isfinite(gimbal_pitch_dps_rx))
		gimbalControl.GimbalEstimate.pitch_angular_velocity_dps = gimbal_pitch_dps_rx;

	if (shit_delay_count < 200)
		shit_delay_count++;
}

/*---------------------------------------------------------------------------闭环控制-----------------------------------------------------------------------------------------*/

/**
 * @brief 云台yaw轴闭环控制（MIT力矩/速度指令输出）
 */
void GimbalControlUpdate(void)
{
	/* yaw控制已搬迁至上板 */
	if(CONTROL_STOP == pDecisionAO->ctrl_terminal || shit_delay_count < 30)
		gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
}

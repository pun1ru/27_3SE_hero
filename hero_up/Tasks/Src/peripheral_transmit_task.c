#include "tim.h"
#include "usart.h"
#include "general_task_include.h"
#include "CAN_driver.h"
#include "LK_driver.h"
#include "../../PrivateDrivers/Board2Borad/Board2Board.h"
#include "judge_receive.h"
#include "DMJ4310.h"
#include "bsp_dwt.h"
#include "dm_imu.h"
#include "LK_485_driver.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"
int count=0;
extern GimbalControl gimbalControl;
extern const GimbalControl* _gimbalControl;
extern const NormRemoteCmd* _normRemoteCmd;
extern uint8_t dt7RecBuffer[18U];	//dt7原始数据接收区
extern uint8_t VT3RecBuffer[21U];//VT13遥控器接收
uint8_t double_mcu_frame[MCU_FRAME_LEN];//双板通信帧
uint8_t uart10_tx_complete=1;
 uint32_t StateCount=0;
 uint8_t tx[8] = {0};

	/* 调试零值宏: 注释=正常输出, 取消注释=强制该组电机零力矩 */
	//#define ZERO_YAW
	//#define ZERO_PITCH
	//#define ZERO_FRIC
/* IMU数据快速发送：IMUTask调用，PoseUpdateFromIMU后立即发出，消除任务调度延迟 */
volatile uint32_t b2b_pose_tx_cnt = 0;  /* 0x228 GIMBAL_POSE 发送计数 */
void RS485_SendIMU(void)
{
	b2b_pose_tx_cnt++;
/* --- B2B CAN: 云台姿态 500Hz → hero_down --- */
extern Pose gimbalPose;
float yaw_f   = gimbalPose.yaw_d;
float pitch_f = gimbalControl.GimbalEstimate.pitch_angle_d;
float yaw_dps   = gimbalPose.yaw_radps * 57.29578f;
float pitch_dps = gimbalPose.pitch_radps * 57.29578f;

extern WorldGimbal worldGimbal;
if (worldGimbal.enable && _robotState->sniper == SNIPER_ON) {
    yaw_f   = worldGimbal.WorldGimbalEstimate.world_yaw_deg;
    pitch_f = worldGimbal.WorldGimbalEstimate.world_pitch_deg;
}

}

void MotorControlCANSend(void)
{
		/* B2B CAN: 云台姿态 0x228 */
		{
			b2b_pose_tx_cnt++;
			extern Pose gimbalPose;
			extern WorldGimbal worldGimbal;
			extern DMJ4310MotorRec DMyawMotorRec;
			extern float yaw_dm_forward_offset_rad;

			/* yaw: 世界系优先, 否则始终发 DM 编码器（下板底盘跟随需要编码器值） */
			float yaw_f;
			if (worldGimbal.enable && _robotState->sniper == SNIPER_ON)
				yaw_f = worldGimbal.WorldGimbalEstimate.world_yaw_deg;
			else
				yaw_f = AngleLimit((DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f, -180.0f, 180.0f);

			/* pitch 来源: 世界系 > 编码器/IMU */
			float pitch_f;
			if (worldGimbal.enable && _robotState->sniper == SNIPER_ON)
				pitch_f = worldGimbal.WorldGimbalEstimate.world_pitch_deg;
			else
				pitch_f = gimbalControl.GimbalEstimate.pitch_angle_d;

			float yaw_dps   = gimbalControl.GimbalEstimate.yaw_angular_velocity_dps;
			float pitch_dps = gimbalControl.GimbalEstimate.pitch_angular_velocity_dps;

			B2BSendGimbalPose(yaw_f, pitch_f, yaw_dps, pitch_dps);
		}

	/* B2B 下行保护：下板信号丢失 → 停摩擦轮 + pitch零力矩 */
	B2B_DownAliveCheck();
	if (!g_b2b_down_valid) {
		CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
		CANTransmit_I16(&hfdcan2, 0x1FF, 0,0,0,0);
		uint8_t sadata[8] = {0x94};
		LK_iqControl(sadata, 0);
		CANTransmit_U8(&hfdcan1, 0x141, sadata);
		return;
	}

#ifdef ZERO_FRIC
		CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
		CANTransmit_I16(&hfdcan2, 0x1FF, 0,0,0,0);
#else
		if(_robotState->ctrl_terminal != CONTROL_STOP)
		{
			CANTransmit_I16(&hfdcan2, 0x200, _shootControl->ShootMotorControl.fric_target_output[LEFT],_shootControl->ShootMotorControl.fric_target_output[RIGHT],_shootControl->ShootMotorControl.fric_target_output[UP], 0);
			CANTransmit_I16(&hfdcan2, 0x1FF, _shootControl->ShootMotorControl.fric_target_output[LEFT1], _shootControl->ShootMotorControl.fric_target_output[RIGHT1],_shootControl->ShootMotorControl.fric_target_output[UP1], 0);
		}
		else
		{
			CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
			CANTransmit_I16(&hfdcan2, 0x1FF, 0,0,0,0);
		}
#endif
   StateCount++;
	 uint8_t sadata[8] = {0x94};
	 switch(StateCount%3)
	 {
		  case 0:
				sadata[0] = 0x92;
				CANTransmit_U8(&hfdcan1, 0x141 , sadata);
			break;
			case 1:
			case 2:
#ifdef ZERO_PITCH
				LK_iqControl(sadata, 0);
#else
				if(CONTROL_STOP != _robotState->ctrl_terminal)
				{
					if(_robotState->sniper==SNIPER_ON||_robotState->joint_mode==ROBOT_JOINT_MODE_CLIMB)
					LK_MultiLoop_angleControl_limited(sadata,pitch_MAX_SPEED,LK_PITCH_HORIZON_ENCODE+gimbalControl.GimbalTargetInput.pitch_angle_d*800);
					else
					LK_iqControl(sadata,_gimbalControl->GimbalMotorControl.pitch_target_output);
				}
				else
					LK_iqControl(sadata, 0);
#endif
				CANTransmit_U8(&hfdcan1, 0x141 , sadata);
	     break;
		}

		/* ---- yaw DM 电机 MIT 控制 (CAN3 0x017, 从下板搬迁) ---- */
		{
			uint8_t yawData[8];
			extern DMJ4310MotorRec DMyawMotorRec;
#ifdef ZERO_YAW
			DM_MITControl(DMyawMotorRec.pos_d, 0.0f, 0.0f, 0.0f, 0.0f, yawData);
#else
			if(CONTROL_STOP == _robotState->ctrl_terminal)
			{
				DM_MITControl(DMyawMotorRec.pos_d, 0.0f, 0.0f, 0.0f, 0.0f, yawData);
			}
			else
			{
				DM_MITControl(0.0f, 0.0f, 0.0f, 0.0f,
					AbsLimiter(gimbalControl.GimbalMotorControl.yaw_target_output, 10.0f), yawData);
			}
#endif
			CANTransmit_U8(&hfdcan3, 0x017, yawData);
		}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	    if (huart->Instance == PITCH_UART.Instance) 
    {
        uart10_tx_complete = 1; 
    }
}
/**
 * @brief LED闪烁实现（RorGorB 呼吸灯）tim5 channel_1 blue channel_2 green channel_3 red
 */
void LEDShow(void)
{
}
/*---------------------------------------------------data_tx_task region-------------------------------------------*/
static void DebugTransmit(void);
static void CanFix(void); 
static void AutoShoot(void);
void DebugTask(void* argument)
{
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count;	
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._debug_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._debug_task = &this_tick_count;
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	while(1)
	{
		/*在这个任务里在放点别的东西*/
		//CanFix();	
		/*任务主进程*/
		#ifdef DEBUG_MSG_ENABLE
			DebugTransmit();
			//AutoShoot();
		#endif
		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		vTaskDelayUntil(&current_tick_count, DEBUG_TASK_PERIOD_SET);
	}
}
extern RobotState robotState;
extern ShootControl shootControl;
void AutoShoot(){
	static int AutoShootCount=0;
	AutoShootCount++;
	if(AutoShootCount%3000==0 && _robotState->ctrl_terminal != CONTROL_STOP){
		robotState.stir_mode = STIR_ANGLE_CONTROL;
	}
	else
		robotState.stir_mode =STIR_LOCK;
}
extern GimbalControl gimbalControl;
 void CanFix(){
	static uint8_t count=0;
	count++;
	switch(count%100){
		case 1:
			if(hfdcan1.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan1);
		case 2:
			if(hfdcan2.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan2);
		case 3:
			if(hfdcan1.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan3);
	}
 }  	
uint8_t debug_data[42];
#include "ekf_imu_solver.h"
 extern Pose gimbalPose;
 extern DJIGMotorRec pitchMotorRec;
 extern DJIGMotorRec fricMotorRec[FRIC_MOTOR_NUM];
 extern IMUUseEKFSolver imuUseEKFSolver;
 extern  IMURecData imuRecData;
 extern GimbalControl gimbalControl;
 extern ext_shoot_data_t ext_shoot_data;
 extern DJIGMotorRec chassisMotorRec[CHASSIS_MOTOR_NUM];
int16_t trans[6];
int cnt;
static void DebugTransmit(void) 
{
	cnt++;
  int16_t speed_rpm1=-fricMotorRec[1].mechanical_speed_rpm;
	debug_data[0] = 0xAA; debug_data[1] = 0xBB;
	memcpy(&debug_data[2], &fricMotorRec[0].mechanical_speed_rpm, 2);
	memcpy(&debug_data[4], &speed_rpm1, 2);
	memcpy(&debug_data[6], &fricMotorRec[2].mechanical_speed_rpm, 2);
	memcpy(&debug_data[8], &fricMotorRec[3].mechanical_speed_rpm, 2);
	memcpy(&debug_data[10], &fricMotorRec[4].mechanical_speed_rpm, 2);
	memcpy(&debug_data[12], &fricMotorRec[5].mechanical_speed_rpm, 2);
	HAL_UART_Transmit_DMA(&huart7,debug_data, 2+6*2);

}

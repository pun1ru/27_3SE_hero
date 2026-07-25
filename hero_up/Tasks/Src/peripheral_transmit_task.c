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
	#define ZERO_YAW
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
			/* 0x228 必须与上板控制反馈同一坐标系；保护态 yaw 即上板 IMU yaw。 */
			float yaw_f = gimbalControl.GimbalEstimate.yaw_angle_d;
			float pitch_f = gimbalControl.GimbalEstimate.pitch_angle_d;
			float yaw_dps   = gimbalControl.GimbalEstimate.yaw_angular_velocity_dps;
			float pitch_dps = gimbalControl.GimbalEstimate.pitch_angular_velocity_dps;

			B2BSendGimbalPose(yaw_f, pitch_f, yaw_dps, pitch_dps);
		}

	/* 更新 B2B 下行状态；掉线保护在 yaw 本地控制之后执行。 */
	B2B_DownAliveCheck();

	/* ---- yaw DM 电机 MIT 控制 (CAN3 CMD=0x07, RSP=0x017) ---- */
	{
		extern DMJ4310MotorRec DMyawMotorRec;
		static uint16_t yaw_enable_divider = 199U;
		static uint8_t yaw_protect_maint_slot = 0U;

		if (CONTROL_STOP == _robotState->ctrl_terminal)
		{
			/* 保护态保持零力矩，并参照下板维护节拍执行失能和清错。 */
			DM_MITControl_Send(&hfdcan3, 0x07, DMyawMotorRec.pos_d,
				0.0f, 0.0f, 0.0f, 0.0f);

			if (yaw_protect_maint_slot == 0U)
				lock_motor(&hfdcan3, 0x07);
			else if (yaw_protect_maint_slot == 8U)
				clear_error(&hfdcan3, 0x07);

			yaw_protect_maint_slot = (yaw_protect_maint_slot + 1U) % 16U;
			yaw_enable_divider = 199U;  /* 退出保护后的首周期立即使能。 */
		}
		else
		{
			yaw_protect_maint_slot = 0U;
			if (++yaw_enable_divider >= 200U)
			{
				yaw_enable_divider = 0U;
				start_motor(&hfdcan3, 0x07);
			}

#ifdef ZERO_YAW
			DM_MITControl_Send(&hfdcan3, 0x07, DMyawMotorRec.pos_d,
				0.0f, 0.0f, 0.0f, 0.0f);
#else
			DM_MITControl_Send(&hfdcan3, 0x07, 0.0f, 0.0f, 0.0f, 0.0f,
				AbsLimiter(gimbalControl.GimbalMotorControl.yaw_target_output, 5.0f));
#endif
		}
	}

	/* B2B 下行保护：停摩擦轮和 pitch；yaw 已在上面按本地状态处理。 */
	if (!g_b2b_down_valid)
	{
		uint8_t sadata[8] = {0x94};
		CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
		CANTransmit_I16(&hfdcan2, 0x1FF, 0,0,0,0);
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
		#ifdef DEBUG_MSG_ENABLE
			DebugTransmit();

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
extern volatile float g_b2b_stir_toq;
extern volatile float g_b2b_stir_vel;
extern volatile uint8_t g_b2b_shoot_flag;
extern volatile float g_b2b_bullet_speed;
int16_t trans[6];
static uint32_t cnt;
/* ---- Debug 工具函数 ---- */
static uint8_t crc8_maxim(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}
//#define DEBUG_FRAME_ORIGINAL
#define DEBUG_FRAME_YAW_ADRC
static void DebugTransmit(void)
{
	cnt++;
	/* DebugTask runs at 1 kHz; transmit every other invocation at 500 Hz. */
	if ((cnt % 2U) != 0U)
		return;

#ifdef DEBUG_FRAME_YAW_ADRC
	/* ===== YAW ADRC调试帧 (42B) =====
	 * 发送频率：500 Hz。字段更新频率如下：
	 * - ControlTask数据：IMU通知驱动，标称500 Hz；
	 * - IMU姿态数据：IMUTask更新，标称500 Hz；
	 * - DM编码器数据：FDCAN3响应中断异步更新，此处读取最新值，标称约500 Hz。
	 * [0-1]   0xAA 0xBB  帧头
	 * [2-5]   float       td_x1_deg       TD跟踪位置 (deg, ControlTask 500 Hz)
	 * [6-9]   float       adrc_u          ADRC最终控制量u (ControlTask 500 Hz)
	 * [10-13] float       eso_z3          ESO扰动估计z3 (ControlTask 500 Hz)
	 * [14-17] float       eso_z2          ESO速度估计z2 (deg/s, ControlTask 500 Hz)
	 * [18-21] float       esf_e1_deg      位置误差e1 (deg, ControlTask 500 Hz)
	 * [22-25] float       eso_z1_deg      ESO位置估计z1 (deg, ControlTask 500 Hz)
	 * [26-29] float       yaw_actual_deg  估计yaw角度 (deg, IMUTask 500 Hz)
	 * [30-33] float       yaw_imu_dps     上板IMU yaw角速度 (deg/s, IMUTask 500 Hz)
	 * [34-37] float       yaw_imu_raw_d   上板IMU EKF yaw角度 (deg, IMUTask 500 Hz)
	 * [38-41] float       yaw_enc_raw_d   DM yaw编码器角度 (deg, CAN异步最新值, 未滤波)
	 */
	extern DMJ4310MotorRec DMyawMotorRec;
	extern float yaw_dm_forward_offset_rad;
	extern Pose gimbalPose;
	const ADRC* adrc = &gimbalControl.GimbalMotorControl.yaw_ADRC;
	float td_x1_d    = adrc->td.x1 * 57.29578f;
	float adrc_u     = adrc->u;
	float eso_z3     = adrc->eso.z3;
	float eso_z2     = adrc->eso.z2 * 57.29578f;   /* rad/s→deg/s */
	float esf_e1_d   = adrc->esf.e_1 * 57.29578f;
	float eso_z1_d   = adrc->eso.z1 * 57.29578f;
	float yaw_actual = gimbalControl.GimbalEstimate.yaw_angle_d;
	float yaw_dps    = gimbalPose.yaw_radps * 57.29578f;
	float yaw_imu_d  = gimbalPose.yaw_d;            /* EKF IMU解算yaw */
	float yaw_enc_d  = (float)((double)(DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578);

	debug_data[0] = 0xAA;
	debug_data[1] = 0xBB;
	memcpy(&debug_data[2],  (void*)&td_x1_d,     4);
	memcpy(&debug_data[6],  (void*)&adrc_u,      4);
	memcpy(&debug_data[10], (void*)&eso_z3,      4);
	memcpy(&debug_data[14], (void*)&eso_z2,      4);
	memcpy(&debug_data[18], (void*)&esf_e1_d,    4);
	memcpy(&debug_data[22], (void*)&eso_z1_d,    4);
	memcpy(&debug_data[26], (void*)&yaw_actual,  4);
	memcpy(&debug_data[30], (void*)&yaw_dps,     4);
	memcpy(&debug_data[34], (void*)&yaw_imu_d,   4);
	memcpy(&debug_data[38], (void*)&yaw_enc_d,   4);
	HAL_UART_Transmit_DMA(&huart7, debug_data, 42);
#elif defined(DEBUG_FRAME_ORIGINAL)
	/* ===== 原始帧格式（hero_down 移植） ===== */
	static uint16_t debug_seq = 0;
	/* shoot_flag/弹速来自 B2B 0x224。帧头 2B | 序号 2B | shoot_flag 1B | initial_speed 4B |
	           fric_rpm[6] 12B | pitch_angle 4B | yaw_angle 4B |
	           stir_torque 4B | stir_speed 4B | CRC-8 1B = 38B */
	debug_data[0] = 0xAA;
	debug_data[1] = 0xBB;
	memcpy(&debug_data[2], &debug_seq, 2);
	debug_seq++;
	debug_data[4] = g_b2b_shoot_flag;
	memcpy(&debug_data[5], (void*)&g_b2b_bullet_speed, 4);
	memcpy(&debug_data[9],  (void*)&fricMotorRec[0].mechanical_speed_rpm, 2);
	memcpy(&debug_data[11], (void*)&fricMotorRec[1].mechanical_speed_rpm, 2);
	memcpy(&debug_data[13], (void*)&fricMotorRec[2].mechanical_speed_rpm, 2);
	memcpy(&debug_data[15], (void*)&fricMotorRec[3].mechanical_speed_rpm, 2);
	memcpy(&debug_data[17], (void*)&fricMotorRec[4].mechanical_speed_rpm, 2);
	memcpy(&debug_data[19], (void*)&fricMotorRec[5].mechanical_speed_rpm, 2);
	memcpy(&debug_data[21], (void*)&gimbalControl.GimbalEstimate.pitch_angle_d, 4);
	memcpy(&debug_data[25], (void*)&_gimbalControl->GimbalEstimate.yaw_angle_d, 4);
	memcpy(&debug_data[29], (void*)&g_b2b_stir_toq, 4);
	memcpy(&debug_data[33], (void*)&g_b2b_stir_vel, 4);
	debug_data[37] = crc8_maxim(debug_data, 37);
	HAL_UART_Transmit_DMA(&huart7, debug_data, 38);
#else
	/* 摩擦轮转速调试帧 (14B): 0xAA 0xBB + 6×int16 RPM */
	int16_t speed_rpm1=-fricMotorRec[1].mechanical_speed_rpm;
	debug_data[0] = 0xAA; debug_data[1] = 0xBB;
	memcpy(&debug_data[2], &fricMotorRec[0].mechanical_speed_rpm, 2);
	memcpy(&debug_data[4], &speed_rpm1, 2);
	memcpy(&debug_data[6], &fricMotorRec[2].mechanical_speed_rpm, 2);
	memcpy(&debug_data[8], &fricMotorRec[3].mechanical_speed_rpm, 2);
	memcpy(&debug_data[10], &fricMotorRec[4].mechanical_speed_rpm, 2);
	memcpy(&debug_data[12], &fricMotorRec[5].mechanical_speed_rpm, 2);
	HAL_UART_Transmit_DMA(&huart7,debug_data, 2+6*2);
#endif
}

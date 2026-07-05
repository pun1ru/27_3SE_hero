#include "robot_control_task.h"
#include "tim.h"
#include "usart.h"
#include "general_task_include.h"
#include "CAN_driver.h"
#include "LK_driver.h"
#include "judge_receive.h"
#include "DMJ4310.h"
#include "UI_driver.h"
#include "bsp_dwt.h"
#include "dm_imu.h"
#include "LK_485_driver.h"
#include "peripheral_receive_task.h"
#include "jointControl.h"
//#include "distance_check.h"
#define MCU_FRAME_LEN (1U+21U+4U+2U+12U)  /* 帧头1B + 遥控21B + yaw 4B + HP 2B + 机体姿态角 12B(pitch/roll/yaw各4B) */
int count=0;
extern JointControl jointControl;
extern GimbalControl gimbalControl;
extern const GimbalControl* _gimbalControl;
extern DMJ4310MotorRec stirMotorRec;
extern const NormRemoteCmd* _normRemoteCmd;
extern volatile float gimbal_yaw_rx_d;           /* RS485接收的实际yaw */
extern volatile float gimbal_yaw_target_rx_d;   /* RS485接收的目标yaw */
extern volatile float gimbal_pitch_target_rx_d;  /* RS485接收的目标pitch */
extern volatile float gimbal_pitch_rx_d;         /* RS485接收的实际pitch */
extern uint32_t multicircle;
extern int64_t circle_angle;
extern uint8_t dt7RecBuffer[18U];	//dt7原始数据接收区
extern uint8_t VT3RecBuffer[21U];//VT3遥控器接收
extern int crawler_rotate_flag;
extern JointBodyState g_joint_body_state_body_dbg;  /* 机体坐标系下姿态角（已从IMU系旋转到机体系） */
extern DMJ4310MotorRec DMyawMotorRec;
extern float yaw_dm_forward_offset_rad;
uint8_t uart10_tx_complete=1;
uint8_t double_mcu_frame[MCU_FRAME_LEN];//双板通信帧
float joint_target[4]={};

 int first_flag=1;
 uint32_t StateCount=0;
/* ALLHighFreqCal 已合并到 GimbalControlUpdate */
 float angle_temp=0;
void MotorControlCANSend(void)
{
		
	/*注意同一总线帧合并*/
	static uint8_t send_count;
	count++;
	
	
	//can分两个
	uint8_t adata[8]={0};
	switch(count%7){
		case 6://达秒电机使能
		if(_robotState->ctrl_terminal != CONTROL_STOP){
			start_motor(&hfdcan1, 0x01);
			start_motor(&hfdcan1, 0x02);
			start_motor(&hfdcan1, 0x03);
			start_motor(&hfdcan1, 0x04);
			start_motor(&hfdcan1, 0x07);
			start_motor(&hfdcan1, 0x08);//只使能拨盘
		}
		if(_robotState->ctrl_terminal == CONTROL_STOP){
		lock_motor(&hfdcan1, 0x01);
		lock_motor(&hfdcan1, 0x02);
		lock_motor(&hfdcan1, 0x03);
		lock_motor(&hfdcan1, 0x04);
		lock_motor(&hfdcan1, 0x07);
		lock_motor(&hfdcan1, 0x08);
		clear_error(&hfdcan1,0x07);
		clear_error(&hfdcan1,0x01);
		clear_error(&hfdcan1,0x02);
		clear_error(&hfdcan1,0x03);
		clear_error(&hfdcan1,0x04);
		clear_error(&hfdcan1,0x07);
		clear_error(&hfdcan1,0x08);
		}
			/*拨盘电机作一帧*/
			//改成can1 master 0x18 
			
		break;
		case 1:
		case 3:
		case 5:
			if(_robotState->ctrl_terminal != CONTROL_STOP){
				//轮电机
    //   CANTransmit_I16(&hfdcan2, 0x200, _chassisControl->WheelMotorControl.target_motor_output[0], _chassisControl->WheelMotorControl.target_motor_output[1],\
    //   _chassisControl->WheelMotorControl.target_motor_output[2],_chassisControl->WheelMotorControl.target_motor_output[3]);
			  CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);//注释	
			}
			if(_robotState->ctrl_terminal == CONTROL_STOP)
			CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
			break;
		case 0:
		case 2:
		case 4:
		if(_robotState->ctrl_terminal != CONTROL_STOP)
		{
		       if(_robotState->sniper==SNIPER_OFF)
				{
		     	//  DM_MITControl_Send(&hfdcan1,0x01,jointControl.JointMotorControl.mit_p[LEG_LF],0,jointControl.JointMotorControl.mit_Kp[LEG_LF],jointControl.JointMotorControl.mit_Kd[LEG_LF],jointControl.JointMotorControl.mit_Tff[LEG_LF]);//-1.9
			 	//  DM_MITControl_Send(&hfdcan1,0x02,jointControl.JointMotorControl.mit_p[LEG_RF],0,jointControl.JointMotorControl.mit_Kp[LEG_RF],jointControl.JointMotorControl.mit_Kd[LEG_RF],jointControl.JointMotorControl.mit_Tff[LEG_RF]);//1.55
			 	//  DM_MITControl_Send(&hfdcan1,0x03,jointControl.JointMotorControl.mit_p[LEG_RB],0,jointControl.JointMotorControl.mit_Kp[LEG_RB],jointControl.JointMotorControl.mit_Kd[LEG_RB],jointControl.JointMotorControl.mit_Tff[LEG_RB]);//2.09
			 	//  DM_MITControl_Send(&hfdcan1,0x04,jointControl.JointMotorControl.mit_p[LEG_LB],0,jointControl.JointMotorControl.mit_Kp[LEG_LB],jointControl.JointMotorControl.mit_Kd[LEG_LB],jointControl.JointMotorControl.mit_Tff[LEG_LB]);//-2.29
			 	 DM_MITControl_Send(&hfdcan1,0x01,0,0,0,0,0);
				 DM_MITControl_Send(&hfdcan1,0x02,0,0,0,0,0);	
				 DM_MITControl_Send(&hfdcan1,0x03,0,0,0,0,0);
				 DM_MITControl_Send(&hfdcan1,0x04,0,0,0,0,0);//注释
			    }
				else if(_robotState->sniper==SNIPER_ON)
				{
					DM_MITControl_Send(&hfdcan1,0x01,0,0,0,0,0);
					DM_MITControl_Send(&hfdcan1,0x02,0,0,0,0,0);	
					DM_MITControl_Send(&hfdcan1,0x03,0,0,0,0,0);
					DM_MITControl_Send(&hfdcan1,0x04,0,0,0,0,0);
				}
				if(_robotState->ctrl_terminal == CONTROL_STOP)
				{
				DM_MITControl_Send(&hfdcan1,0x01,0,0,0,0,0);
				DM_MITControl_Send(&hfdcan1,0x02,0,0,0,0,0);	
				DM_MITControl_Send(&hfdcan1,0x03,0,0,0,0,0);
				DM_MITControl_Send(&hfdcan1,0x04,0,0,0,0,0);	
			}			
		break;
		}
	}
	 /* ALLHighFreqCal merged into GimbalControlUpdate */
   StateCount++;
	 switch(StateCount%3)
	 {
		  case 0:
		if(_robotState->ctrl_terminal != CONTROL_STOP){
		start_motor(&hfdcan2, 0x05);//
		start_motor(&hfdcan2, 0x06);//
		// lock_motor(&hfdcan2, 0x05);
		// lock_motor(&hfdcan2, 0x06);
		// clear_error(&hfdcan2,0x05);
		// clear_error(&hfdcan2,0x06);//注释
			
	
		}
		if(_robotState->ctrl_terminal == CONTROL_STOP){
		lock_motor(&hfdcan2, 0x05);
		lock_motor(&hfdcan2, 0x06);
		clear_error(&hfdcan2,0x05);
		clear_error(&hfdcan2,0x06);
		}		
			break;
			case 1:
			case 2:
			
			if(_robotState->ctrl_terminal != CONTROL_STOP)
			{
				  DM_MITControl_Send(&hfdcan1,0x07,
					gimbalControl.GimbalMotorControl.mit.p,
					gimbalControl.GimbalMotorControl.mit.v,
					gimbalControl.GimbalMotorControl.mit.Kp,
					gimbalControl.GimbalMotorControl.mit.Kd,
					gimbalControl.GimbalMotorControl.mit.Tff);
				//DM_MITControl_Send(&hfdcan1,0x07,0,0,0,0,0);//注释
			}
			if(_robotState->ctrl_terminal == CONTROL_STOP)			
			DM_MITControl_Send(&hfdcan1,0x07,0,0,0,0,0);	
	    break;
		}
	switch(StateCount%3)
	{
		case 0:
	 ctrl_motor2(&hfdcan1, GMJ4310MOTOR_ID, _shootControl->ShootTargetInput.stir_all_target_pos_rad, _shootControl->ShootTargetInput.stir_target_vol);
		break;
	  case 1:
	  //超电
	CANTransmit_I16(&hfdcan2,0x2ff,0,0,ext_game_robot_status.chassis_power_limit-1,0);
		
		break;
		case 2:
			if (_robotState->ctrl_terminal == CONTROL_STOP)
			{
				DM_MITControl_Send(&hfdcan2, 0x05, 0, 0, 0, 0, 0);
				DM_MITControl_Send(&hfdcan2, 0x06, 0, 0, 0, 0, 0);
			}
			else if (crawler_rotate_flag!=0)
			{
				DM_MITControl_Send(&hfdcan2, 0x05, 0, 0, 0, 0, -3.1f*crawler_rotate_flag);
				DM_MITControl_Send(&hfdcan2, 0x06, 0, 0, 0, 0, 3.1f*crawler_rotate_flag);
			}
			else
			{
				DM_MITControl_Send(&hfdcan2, 0x05, 0, 0, 0, 0, 0);
				DM_MITControl_Send(&hfdcan2, 0x06, 0, 0, 0, 0, 0);
			}
		break;
	}		
    HAL_GPIO_WritePin(RS485_MASTER_DE_GPIO_Port, RS485_MASTER_DE_Pin, GPIO_PIN_SET);//拉低master的de引脚电平,通信用
		if (_normRemoteCmd->remote_source == DT7)//dt7遥控器
    	{
		double_mcu_frame[0]=0x07;
		memcpy(double_mcu_frame+1,dt7RecBuffer,18U);
		}
		else if (_normRemoteCmd->remote_source == VT13)//vt13遥控器
    {
		double_mcu_frame[0]=0x03;
		memcpy(double_mcu_frame+1,VT3RecBuffer,21U);
		}
		else if (_normRemoteCmd->remote_source == ERROR_RECEIVE)
    {
		double_mcu_frame[0]=0x88;
		double_mcu_frame[1]=0x88;//没收到遥控器数据帧
		}
		/* 强制使用DM yaw编码器角度发送，不依赖gimbalControl.GimbalEstimate.yaw_angle_d */
		float yaw_enc_deg_tx = AngleLimit((DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f, -180.0f, 180.0f);
		memcpy(double_mcu_frame + 1U + 21U, &yaw_enc_deg_tx, sizeof(yaw_enc_deg_tx));
		memcpy(double_mcu_frame + 1U + 21U + 4U, &ext_game_robot_status.current_HP, sizeof(ext_game_robot_status.current_HP));
	/* 机体坐标系姿态角（已从IMU系经JointRotateImuStateToBody旋转到机体系，单位：度） */
	memcpy(double_mcu_frame + 1U + 21U + 4U + 2U,      &g_joint_body_state_body_dbg.pitch_d, sizeof(float));  /* 机体pitch */
	memcpy(double_mcu_frame + 1U + 21U + 4U + 2U + 4U, &g_joint_body_state_body_dbg.roll_d,  sizeof(float));  /* 机体roll  */
	memcpy(double_mcu_frame + 1U + 21U + 4U + 2U + 8U, &g_joint_body_state_body_dbg.yaw_d,   sizeof(float));  /* 机体yaw   */
	HAL_UART_Transmit_DMA(&MASTER_485_UART,double_mcu_frame, MCU_FRAME_LEN);
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
static void SaoPin(void);
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
float i=0;
float j=0;
int t=0;
float sin_test=0;
extern GimbalControl gimbalControl;
 void CanFix(){
	static uint8_t count=0;
	count++;
	//uint8_t now=count%3;
	switch(count%100){
		case 1:
			if(hfdcan1.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan1);
		case 2:
			if(hfdcan2.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan2);
		case 3:
			if(hfdcan3.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan3);
	}
 }
uint8_t debug_data[42];
extern float distance;
extern DataFromJudge bulletSpeed;
extern float predict_speed0;
 float lastspeed=0;
#include <stdio.h> 
#include "ekf_imu_solver.h"
 #include "shoot_speed_best_contrl.h"
 float temp,temp2,temp3,temp4,temp5,temp6;
 extern float mardio_speed;
 char tx_buffer[50]; // 定义一个50字节的缓冲区，足够存放格式化后的字符串
 extern float current_fric_speed;
 extern Pose gimbalPose;
 extern DJIGMotorRec pitchMotorRec;
 float k=200;
 extern DJIGMotorRec fricMotorRec[FRIC_MOTOR_NUM];
 extern IMUUseEKFSolver imuUseEKFSolver;
 extern  IMURecData imuRecData;
 extern GimbalControl gimbalControl;
 extern volatile int16_t gimbal_fric_rpm_rx;
 extern volatile int16_t gimbal_fric_rpm_rx_arr[6];
 extern ext_shoot_data_t ext_shoot_data;
 extern ext_power_heat_data_t ext_power_heat_data;
 extern DJIGMotorRec chassisMotorRec[CHASSIS_MOTOR_NUM];
 extern SuperCapacity superCapacity;
 extern JointBodyState g_joint_body_state_body_dbg;
 extern ChassisControl chassisControl;
 extern JointControl jointControl;
//gimbalControl.GimbalEstimate.pitch_angular_velocity_dps
extern float w_d;
int16_t trans[6];
int cnt;

/* CRC-8/poly 0x07 (MAXIM) 逐位法校验 */
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

static void DebugTransmit(void)
{
#if 0
	/* ===== 原始帧格式（保留） ===== */
	static uint16_t debug_seq = 0;
	cnt++;
	/* 帧格式:
	 * 偏移  长度  类型    说明
	 * 0-1    2   uint8   帧头 0xAA 0xBB
	 * 2-3    2   uint16  帧序号(0→65535循环,小端)
	 * 4      1   uint8   shoot_flag 射击状态标志位
	 * 5-8    4   float   ext_shoot_data.initial_speed 弹丸初速(m/s)
	 * 9-10   2   int16   fric_rpm[0] 摩擦轮0转速(RPM)
	 * 11-12  2   int16   fric_rpm[1] 摩擦轮1转速(RPM)
	 * 13-14  2   int16   fric_rpm[2] 摩擦轮2转速(RPM)
	 * 15-16  2   int16   fric_rpm[3] 摩擦轮3转速(RPM)
	 * 17-18  2   int16   fric_rpm[4] 摩擦轮4转速(RPM)
	 * 19-20  2   int16   fric_rpm[5] 摩擦轮5转速(RPM)
	 * 21-24  4   float   pitch_angle RS485云台板下发实际pitch(度)
	 * 25-28  4   float   yaw_angle   闭环控制用yaw估计值(度)
	 * 29-32  4   float   stir_torque 拨盘扭矩(解析值 Nm)
	 * 33-36  4   float   stir_speed  拨盘速度(解析值 rad/s)
	 * 37     1   uint8   CRC-8(poly 0x07)校验,覆盖字节0-37
	 */
	debug_data[0] = 0xAA;
	debug_data[1] = 0xBB;

	/* bytes 2-3: 帧序号 (uint16, 小端) */
	memcpy(&debug_data[2], &debug_seq, 2);
	debug_seq++;

	/* byte 4: shoot_flag */
	debug_data[4] = (uint8_t)_shootControl->ShootTargetInput.shoot_flag;

	/* bytes 5-8: initial_speed */
	memcpy(&debug_data[5], &ext_shoot_data.initial_speed, 4);

	/* bytes 9-20: fric_rpm[0..5] — 直接使用volatile源, int16_t */
	memcpy(&debug_data[9],  (void*)&gimbal_fric_rpm_rx_arr[0], 2);
	memcpy(&debug_data[11], (void*)&gimbal_fric_rpm_rx_arr[1], 2);
	memcpy(&debug_data[13], (void*)&gimbal_fric_rpm_rx_arr[2], 2);
	memcpy(&debug_data[15], (void*)&gimbal_fric_rpm_rx_arr[3], 2);
	memcpy(&debug_data[17], (void*)&gimbal_fric_rpm_rx_arr[4], 2);
	memcpy(&debug_data[19], (void*)&gimbal_fric_rpm_rx_arr[5], 2);

	/* bytes 21-24: pitch_angle (RS485云台板下发的实际pitch)(度) */
	memcpy(&debug_data[21], (void*)&gimbal_pitch_rx_d, 4);

	/* bytes 25-28: yaw_angle (闭环控制用yaw估计值)(度) */
	memcpy(&debug_data[25], (void*)&_gimbalControl->GimbalEstimate.yaw_angle_d, 4);

	/* bytes 29-32: stir_torque (拨盘扭矩, 解析值 Nm) */
	memcpy(&debug_data[29], &stirMotorRec.toq, 4);

	/* bytes 33-36: stir_speed (拨盘速度, 解析值 rad/s) */
	memcpy(&debug_data[33], &stirMotorRec.vel_radps, 4);

	/* byte 37: CRC-8, 覆盖字节0-37 */
	debug_data[37] = crc8_maxim(debug_data, 37);

	HAL_UART_Transmit_DMA(&huart7, debug_data, 38);

#else
	/* ===== yaw ADRC调试帧 =====
^I * TD.x1         TD跟踪位置(度)
^I * ESO.z1        ESO估计位置(度)
^I * ESO.z3        ESO扰动估计
^I * ADRC.u        LADRC控制输出
^I * yaw_target    yaw目标角(度)
^I * yaw_raw       yaw原始反馈(度)
	 */
	debug_data[0] = 0xAA;
	debug_data[1] = 0xBB;

	float td_x1_deg  = gimbalControl.GimbalMotorControl.yaw_ADRC.td.x1  * (180.0f / 3.141592f);
	float eso_z1_deg = gimbalControl.GimbalMotorControl.yaw_ADRC.eso.z1 * (180.0f / 3.141592f);
	memcpy(&debug_data[2],  (void*)&td_x1_deg, 4);
	memcpy(&debug_data[6],  (void*)&eso_z1_deg, 4);
	memcpy(&debug_data[10], (void*)&gimbalControl.GimbalMotorControl.yaw_ADRC.eso.z3, 4);
	memcpy(&debug_data[14], (void*)&gimbalControl.GimbalMotorControl.yaw_ADRC.u, 4);
	memcpy(&debug_data[18], (void*)&gimbalControl.GimbalTargetInput.yaw_angle_d, 4);
	memcpy(&debug_data[22], (void*)&_gimbalControl->GimbalEstimate.yaw_angle_d, 4);

	HAL_UART_Transmit_DMA(&huart7, debug_data, 26);
#endif
}
/*---------------------------------------------------UI region-------------------------------------------*/
UIframe_t UIframe;
UIframe_t* _UIframe = &UIframe;

void idObtain(uint16_t* receiver_id, uint16_t* sender_id); void UiOperation();
void UiInit(uint32_t const * ui);
void FrameUpdate(uint32_t * ui);

void UIOperationTask(void* argument)
{
	static uint32_t last_tick_count, current_tick_count, this_tick_count;	
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._ui_operation_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._ui_operation_task = &this_tick_count;
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	while(1)
	{
	
		/*任务主进程*/
		/* 降低发送频率至 ~27.8Hz（3ms×12=36ms），满足裁判系统0x0301的30Hz上限 */
		if(task_counter % 12 == 0)
			UiOperation();

		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		vTaskDelayUntil(&current_tick_count, UI_OPERATION_TASK_PERIOD_SET);
	}
}

/**
 * @brief  获取发送端，接收端id
 * @note   
 * @param  SenderID发送者ID ReceiverID接收者ID
 * @retval None
 */
void idObtain(uint16_t* receiver_id, uint16_t* sender_id)
{
	if(ext_game_robot_status.robot_id != 0)
		{
			*sender_id= ext_game_robot_status.robot_id; //发送方id
			
			if(ext_game_robot_status.robot_id < 100)
				*receiver_id = (ext_game_robot_status.robot_id | 0x0100);
			else
				*receiver_id = ext_game_robot_status.robot_id + 256;
		}
}

/**
 * @brief  ui绘制函数
 * @note   
 * @param  none
 * @retval None
 */
void UiOperation()
{
	static uint32_t ui = 0;
	static uint8_t InitFlag = 0;	
	if(ui %200 < 12)
		UiInit(&ui);
	else
		FrameUpdate(&ui);		
	ui++;

}



/**
* @brief  ui初始化，操作类型为OperateAdd
 * @note   除字符类型外，其他类型均把全部图形画完再发送，字符类型画一个发一个
 * @param  none
 * @retval None
 */
int delta=27;
void UiInit(uint32_t const * ui)
{
	uint8_t* uname;//uint8_t数组，必须三字节
	uint16_t subcontent_id = 0;
	uint8_t TransmitOk = 0;
	
	uint16_t sender_id, receiver_id;
	idObtain(&receiver_id, &sender_id);
	
	uint8_t index = *ui % 12;
	switch(index)
	{
		case 0:
		{
			//图层六，云台目标/实际角度 + aim检测（合并一帧）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_gim;
			char gim_text[48];
			const char* aim_status = ((fabs((double)gimbal_yaw_target_rx_d) > 0.01) && (fabs((double)gimbal_pitch_target_rx_d) > 0.01)) ? "OK" : "Ohno";
			
			uname = (unsigned char*)"gim";
			snprintf(gim_text, sizeof(gim_text), "TY:%.1f Y:%.1f\nTP:%.1f P:%.1f\n%s",
				(double)gimbal_yaw_target_rx_d,
				(double)_gimbalControl->GimbalEstimate.yaw_angle_d,
				(double)gimbal_pitch_target_rx_d,
				(double)gimbal_pitch_rx_d,
				aim_status);
			DrawChar(&char_gim, OperateAdd, 1550, 600, uname, 2, 20, 6, (uint8_t*)gim_text, (uint8_t)strlen(gim_text), ColorYellow);
			
			CharacterToUIframe(&char_gim, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;   
		}
		
		case 1:
		{
			//图层三
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3 = {};
			
			//电容电压值
			uname = (unsigned char*)"cvo";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					OperateAdd, 980, 200, uname, 2, 20, 3, _superCapacity->cap_volt * 1000, ColorGreen);
			
			//激光测距值
			uname = (unsigned char*)"dos";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[2],\
					OperateAdd, 980, 830, uname, 2, 20, 3, _distance_check->distance_check_translate.distance_select * 1000, ColorGreen);
			
			
			//前哨站准星 我也不知道，
				uname = (unsigned char*)"qia";
				DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
						 OperateAdd, 960, 900, 960, 200, uname, 1, 3, ColorCyan);
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 2:
		{
			//图层六，鼠标锁定状态（sniper下pitch/yaw锁定）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			ext_interaction_character_t char1;
			if(_robotState->mouse_fix == MOUSE_FIX_ON)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateAdd, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:on ", 13, ColorAmaranth);
			}
			else if(_robotState->mouse_fix == MOUSE_FIX_OFF)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateAdd, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:off", 13, ColorGreen);
			}
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
		
			break;
		}
		
		case 3:
		{
			//图层六，sniper模式显示（STAIR上方）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_snp;
			if(_robotState->sniper == SNIPER_ON)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateAdd, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:ON ", 10, ColorAmaranth);
			}
			else if(_robotState->sniper == SNIPER_OFF)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateAdd, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:OFF", 10, ColorGreen);
			}
			
			CharacterToUIframe(&char_snp, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
		
			break;
		}
		
		case 4:
		{
			//图层六，上台阶状态（边缘显示，不遮挡主视角）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char2;
			
			uname = (unsigned char*)"std";
			switch(_robotState->stand_mode)
			{
				case ROBOT_STAND_MODE_NORMAL:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:NORMAL    ", 16, ColorGreen);
					break;
				case ROBOT_STAND_MODE_PRE_STAIR:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE       ", 16, ColorYellow);
					break;
				case ROBOT_STAND_MODE_STAIR_UP:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:UP        ", 16, ColorAmaranth);
					break;
				case ROBOT_STAND_MODE_PRE_DOWN_STAIR:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE_DOWN  ", 16, ColorOrange);
					break;
				default:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:???       ", 16, ColorOrange);
					break;
			}
			
			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			break;
		}
		
		case 5:
		{
			//图层六，电容电压
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			ext_interaction_character_t char1;
			
			uname = (unsigned char *)"cav";
			DrawChar(&char1, OperateAdd, 800, 200, uname, 2, 20, 6, (uint8_t*)"CapVot:   ", 10, ColorYellow);
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		
		case 6:
		{
			//图层六
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3 = {};
			//准星1
			uname = (unsigned char*)"ap1";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					 OperateAdd, 850-delta,  540, 970-delta, 540, uname, 1, 6, ColorGreen);
			//清除已删除的旧图形残留（激光落点、自动补偿条）
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
					 OperateDelete, 0, 0, 0, 0, (uint8_t*)"hen", 1, 6, ColorYellow);
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[3],\
					 OperateDelete, 0, 0, 0, 0, (uint8_t*)"shu", 1, 6, ColorAmaranth);
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[4],\
					 OperateDelete, 0, 0, 0, 0, (uint8_t*)"ap3", 1, 6, ColorGreen);
				
			//准星2
//			uname = (unsigned char*)"ap2";
//			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
//					 OperateAdd, 850-delta, 432, 970-delta, 432, uname, 1, 6, ColorGreen);//
			
			//电容条
			uint8_t color;
			
			if(_superCapacity->cap_volt < 12)
				color = ColorOrange;
			else if(_superCapacity->cap_volt < 18)
				color = ColorYellow;
			else 
				color = ColorGreen;
			
			uint32_t Power_Line = (uint32_t)(((_superCapacity->cap_volt - 12) / 12.0f) * 600); //电容条
			
			uname = (unsigned char *)"cap";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[2],\
					 OperateAdd, 700, 100, (700 + Power_Line), 100, uname, 40, 6, color);
			
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 7:
		{//吊射模式显示
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"dio";
				if(_robotState->follow==FOLLOW_ON)//吊射模式
				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
			else if(_robotState->follow==FOLLOW_OFF)//普通模式
				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off ", 10, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
		    break;
		}
		case 8:
		{
			//图层七，摩擦轮开关标志
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char1;
			
			uname = (unsigned char*)"fri";
			if(_robotState->fric_mode == FRIC_ON)
				DrawChar(&char1, OperateAdd, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: On  ", 10, ColorAmaranth);
			else
				DrawChar(&char1, OperateAdd, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: Close", 11, ColorGreen);
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		case 9:
		{
			//图层九

			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3 = {};
			
			uname = (unsigned char*)"aaa";
			DrawRect(&figure3.ext_interaction_figure_3.interaction_figure[0],\
			OperateAdd, 685, 240, 1235, 710, uname, 5, 8, ColorAmaranth);
			
			static uint8_t draw_flag = 0;
			if(_robotState->chassis_mode == CHASSIS_SEPARATE && !draw_flag)
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateAdd, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateAdd, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 1;
		    }
			else if( _robotState->chassis_mode != CHASSIS_SEPARATE && draw_flag )
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateDelete, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateDelete, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 0;
			}
			
			uname = (unsigned char*)"fff";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[3],\
			         OperateAdd, 910-delta, 540, 910-delta, 380, uname, 1, 8, ColorGreen);
			
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		
		case 10:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"tuy";
			if(_robotState->capacity_mode == NO_CAPACITY)
				DrawChar(&char3, OperateAdd, 200, 730, uname, 2, 20, 6, (uint8_t*)"LOW POWER", 9, ColorAmaranth);
			else if(_robotState->capacity_mode == CAPACITY)
				DrawChar(&char3, OperateAdd, 200, 730, uname, 2, 20, 6, (uint8_t*)"NORMAL POWER", 12, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 11:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"fuk";
			if(_robotState->chassis_mode == CHASSIS_REVOLVE)
				DrawChar(&char3, OperateAdd, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin On", 7, ColorAmaranth);
			else
				DrawChar(&char3, OperateAdd, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin Off", 8, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
//			
			break;
		}
		
//		case 11:
//			{//吊射模式显示
//			subcontent_id = 0x0110;
//			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
//			ext_interaction_character_t char3;
//			
//			uname = (unsigned char*)"dio";
//				if(_robotState->follow==FOLLOW_ON)//吊射模式
//				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
//			else if(_robotState->follow==FOLLOW_OFF)//普通模式
//				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off ", 10, ColorGreen);
//			
//			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
//			UIframeTransmit(_UIframe->data, subcontent_id);
//			UIframeClear(_UIframe->data);
//			
//			break;
//		}
//		
//		case 12:
//		{
////			subcontent_id = 0x0110;
////			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
////			ext_interaction_character_t char2;
////			
////			uname = (unsigned char*)"dis";
////			DrawChar(&char2, OperateAdd, 1080, 800, uname, 2, 20, 6, (uint8_t*)"DISTANCE:    	", 10, ColorYellow);
////			
////			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
////			UIframeTransmit(_UIframe->data, subcontent_id);
////			UIframeClear(_UIframe->data);
//		
//			break;
//		}
			
		default:
			break;
	}
	
}


/**
* @brief  ui更新，操作类型为OperateChange
 * @note   除字符类型外，其他类型均把全部图形画完再发送，字符类型画一个发一个
 * @param  none
 * @retval None
 */
void FrameUpdate(uint32_t * ui)
{
	uint8_t* uname;//uint8_t数组，必须三字节
	uint16_t subcontent_id = 0;
	uint8_t TransmitOk = 0;
	
	uint16_t sender_id, receiver_id;
	idObtain(&receiver_id, &sender_id);
	
	uint8_t index = *ui % 12;
	switch(index)
	{
		case 0:
		{
			//图层六，云台目标/实际角度 + aim检测（合并一帧）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_gim;
			char gim_text[48];
			const char* aim_status = ((fabs((double)gimbal_yaw_target_rx_d) > 0.01) && (fabs((double)gimbal_pitch_target_rx_d) > 0.01)) ? "OK" : "Ohno";
			
			uname = (unsigned char*)"gim";
			snprintf(gim_text, sizeof(gim_text), "TY:%.1f Y:%.1f\nTP:%.1f P:%.1f\n%s",
				(double)gimbal_yaw_target_rx_d,
				(double)_gimbalControl->GimbalEstimate.yaw_angle_d,
				(double)gimbal_pitch_target_rx_d,
				(double)gimbal_pitch_rx_d,
				aim_status);
			DrawChar(&char_gim, OperateChange, 1550, 600, uname, 2, 20, 6, (uint8_t*)gim_text, (uint8_t)strlen(gim_text), ColorYellow);
			
			CharacterToUIframe(&char_gim, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 1:
		{
			//图层三
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3= {};
			
			//电容电压值
			uname = (unsigned char*)"cvo";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					OperateChange, 980, 200, uname, 2, 20, 3, _superCapacity->cap_volt * 1000, ColorGreen);
			
			//激光测距值
			uname = (unsigned char*)"dos";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[3],\
					OperateChange, 980, 830, uname, 2, 20, 3, _distance_check->distance_check_translate.distance_select * 1000, ColorGreen);
			
			//前哨站准星
				uname = (unsigned char*)"qia";
				DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
						 OperateChange, 960, 900, 960, 200, uname, 1, 3, ColorCyan);
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 2:
		{
			//图层六，鼠标锁定状态
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			ext_interaction_character_t char1;
			if(_robotState->mouse_fix == MOUSE_FIX_ON)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateChange, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:on ", 13, ColorAmaranth);
			}
			else if(_robotState->mouse_fix == MOUSE_FIX_OFF)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateChange, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:off", 13, ColorGreen);
			}
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 3:
		{
			//图层六，sniper模式显示（STAIR上方）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_snp;
			if(_robotState->sniper == SNIPER_ON)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateChange, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:ON ", 10, ColorAmaranth);
			}
			else if(_robotState->sniper == SNIPER_OFF)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateChange, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:OFF", 10, ColorGreen);
			}
			
			CharacterToUIframe(&char_snp, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 4:
		{
			//图层六，上台阶状态（边缘显示，不遮挡主视角）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char2;
			
			uname = (unsigned char*)"std";
			switch(_robotState->stand_mode)
			{
				case ROBOT_STAND_MODE_NORMAL:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:NORMAL    ", 16, ColorGreen);
					break;
				case ROBOT_STAND_MODE_PRE_STAIR:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE       ", 16, ColorYellow);
					break;
				case ROBOT_STAND_MODE_STAIR_UP:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:UP        ", 16, ColorAmaranth);
					break;
				case ROBOT_STAND_MODE_PRE_DOWN_STAIR:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE_DOWN  ", 16, ColorOrange);
					break;
				default:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:???       ", 16, ColorOrange);
					break;
			}
			
			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			break;
		}
		
		case 5:
		{
			//图层六，电容电压
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char1;
			
			uname = (unsigned char *)"cav";
			DrawChar(&char1, OperateChange, 800, 200, uname, 2, 20, 6, (uint8_t*)"CapVot:   ", 10, ColorYellow);
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 6:
		{
			//图层六
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3= {};
			//准星1
			uname = (unsigned char*)"ap1";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					 OperateChange, 850-delta, 540, 970-delta, 540, uname, 1, 6, ColorGreen);
				
//			uname = (unsigned char*)"ap2";
//			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
//					 OperateChange, 850-delta, 432, 970-delta, 432, uname, 1, 6, ColorGreen);
			
			//电容条
			uint8_t color;
			
			if(_superCapacity->cap_volt < 12)
				color = ColorOrange;
			else if(_superCapacity->cap_volt < 18)
				color = ColorYellow;
			else 
				color = ColorGreen;
			
			int Power_Line = (uint32_t)(((_superCapacity->cap_volt - 12) / 12.0f) * 600); //电容条
			
			uname = (unsigned char *)"cap";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[2],\
					 OperateChange, 700, 100, (700 + Power_Line), 100, uname, 40, 6, color);
			
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 7:
		{//吊射模式显示
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"dio";
				if(_robotState->follow==FOLLOW_ON)//吊射模式
				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
			else if(_robotState->follow==FOLLOW_OFF)//普通模式
				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off", 11, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
//			
			break;
		}
		case 8:
		{
			//图层七，摩擦轮开关标志
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char1;
			
			uname = (unsigned char*)"fri";
			if(_robotState->fric_mode == FRIC_ON)
				DrawChar(&char1, OperateChange, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: On  ", 10, ColorAmaranth);
			else
				DrawChar(&char1, OperateChange, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: Close", 11, ColorGreen);
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 9:
		{
			
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3= {};
			
			uint8_t color;
			if(_upperComputerComm->Receive.aiming_state == 0x33)
				color = ColorGreen;
			else
				color = ColorAmaranth;
			
			//检测自瞄通讯是否ok
			static uint32_t last_counter = 0, this_counter = 0;
			this_counter = _upperComputerComm->rec_counter;
			if(this_counter == last_counter)
				color = ColorBlack;
			last_counter = this_counter;

			uname = (unsigned char*)"aaa";
			DrawRect(&figure3.ext_interaction_figure_3.interaction_figure[0],\
			OperateChange, 695, 240, 1235, 710, uname, 5, 8, color);
			
			static uint8_t draw_flag = 0;
			if(_robotState->chassis_mode == CHASSIS_SEPARATE && !draw_flag)
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateAdd, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateAdd, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 1;
		    }
			else if( _robotState->chassis_mode != CHASSIS_SEPARATE && draw_flag )
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateDelete, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateDelete, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 0;
			}
			else if( _robotState->chassis_mode == CHASSIS_SEPARATE && draw_flag )
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateChange, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateChange, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
			}
			
			uname = (unsigned char*)"fff";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[3],\
			         OperateChange, 910-delta, 540, 910-delta, 380, uname, 1, 8, ColorGreen);
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		
		case 10:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"tuy";
			if(_robotState->capacity_mode == NO_CAPACITY)
				DrawChar(&char3, OperateChange, 200, 730, uname, 2, 20, 6, (uint8_t*)"LOW POWER", 9, ColorAmaranth);
			else if(_robotState->capacity_mode == CAPACITY)
				DrawChar(&char3, OperateChange, 200, 730, uname, 2, 20, 6, (uint8_t*)"NORMAL POWER", 12, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 11:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"fuk";
			if(_robotState->chassis_mode == CHASSIS_REVOLVE)
				DrawChar(&char3, OperateChange, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin On ", 8, ColorAmaranth);
			else
				DrawChar(&char3, OperateChange, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin Off ", 9, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
//		case 11:
//			{//吊射模式显示
//			subcontent_id = 0x0110;
//			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
//			ext_interaction_character_t char3;
//			
//			uname = (unsigned char*)"dio";
//				if(_robotState->follow==FOLLOW_ON)//吊射模式
//				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
//			else if(_robotState->follow==FOLLOW_OFF)//普通模式
//				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off", 11, ColorGreen);
//			
//			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
//			UIframeTransmit(_UIframe->data, subcontent_id);
//			UIframeClear(_UIframe->data);
//			
//			break;
//		}
//		
//		case 12:
//		{
////			subcontent_id = 0x0110;
////			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
////			ext_interaction_character_t char2;
////			
////			uname = (unsigned char*)"dis";
////			DrawChar(&char2, OperateChange, 1080, 800, uname, 2, 20, 6, (uint8_t*)"DISTANCE:    	", 10, ColorYellow);
////			
////			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
////			UIframeTransmit(_UIframe->data, subcontent_id);
////			UIframeClear(_UIframe->data);
//		
//			break;
//		}
		
		default:
			break;
	}
	//需要画继续加，最多加到case 8
}

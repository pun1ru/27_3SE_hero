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
//#include "distance_check.h"
int count=0;
extern GimbalControl gimbalControl;
extern const GimbalControl* _gimbalControl;
extern const NormRemoteCmd* _normRemoteCmd;
extern uint32_t multicircle;
extern int64_t circle_angle;
extern uint8_t dt7RecBuffer[18U];	//dt7原始数据接收区
extern uint8_t VT3RecBuffer[21U];//VT13遥控器接收
uint8_t double_mcu_frame[MCU_FRAME_LEN];//双板通信帧
uint8_t uart10_tx_complete=1;
 //extern IMUUseEKFSolver imuUseEKFSolver;
/**
 * @brief 电机的CAN信号帧发送
 * @note  总线挂载情况：CAN1-(4*M3508底盘电机)+(GM6020yaw电机+电容控制板通信)；CAN2-(GM6020pitch电机+2*M3508摩擦轮电机)+MS4010拨盘电机
 */
 int first_flag=1;
 uint32_t StateCount=0;
 uint8_t tx[8] = {0};

 float angle_temp=0;
 int run_cnt;

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

/* B2B已移至MotorControlCANSend */ // B2BSendGimbalPose(yaw_f, pitch_f, yaw_dps, pitch_dps);
}

void MotorControlCANSend(void)
{
	/* B2B CAN: 云台姿态 0x228 — 从 IMUTask/RS485_SendIMU 移至此处 */
	{
		b2b_pose_tx_cnt++;
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

		/*摩擦轮作一帧，加一点额外保护*/
	if(_robotState->ctrl_terminal != CONTROL_STOP)  {
		//CANTransmit_I16(&hfdcan2, 0x200,600,+600,+600,0);
        		//CANTransmit_I16(&hfdcan2, 0x1FF,-600,-600,+600,0);
       	   CANTransmit_I16(&hfdcan2, 0x200, _shootControl->ShootMotorControl.fric_target_output[LEFT],_shootControl->ShootMotorControl.fric_target_output[RIGHT],_shootControl->ShootMotorControl.fric_target_output[UP], 0);
           CANTransmit_I16(&hfdcan2, 0x1FF, _shootControl->ShootMotorControl.fric_target_output[LEFT1], _shootControl->ShootMotorControl.fric_target_output[RIGHT1],_shootControl->ShootMotorControl.fric_target_output[UP1], 0);
//		       	  CANTransmit_I16(&hfdcan2, 0x200, _shootControl->ShootMotorControl.fric_target_output[LEFT],_shootControl->ShootMotorControl.fric_target_output[RIGHT],0, 0);
//           CANTransmit_I16(&hfdcan2, 0x1FF, 0, 0,_shootControl->ShootMotorControl.fric_target_output[UP1], 0);
//	CANTransmit_I16(&hfdcan2, 0x200,0,0,0,0);    
//    CANTransmit_I16(&hfdcan2, 0x1FF,0,0,0,0);
    }
    else                                       
    {
	CANTransmit_I16(&hfdcan2, 0x200,0,0,0,0);      
    CANTransmit_I16(&hfdcan2, 0x1FF,0,0,0,0);
    }
	static uint8_t send_count;
	
	
	/*大小pitch计算，有些保护写在这里*/
	/* ALLHighFreqCal 已搬迁至 GimbalControlUpdate + EstimateTask */
   StateCount++;
	/*pitch高速计算*/
	 uint8_t sadata[8] = {0x94};//这是为什么来着
	 switch(StateCount%3)
	 {
		  case 0:
					sadata[0] = 0x92;
					CANTransmit_U8(&hfdcan1, 0x141 , sadata);
			break;
			case 1:
			case 2:
				
					 if(CONTROL_STOP != _robotState->ctrl_terminal)
					 {
						 if(_robotState->sniper==SNIPER_ON||_robotState->joint_mode==ROBOT_JOINT_MODE_CLIMB)//吊射模式直接编码器控制
                         //LK_SingleLoop_angleControl_limited(sadata,gimbalControl.GimbalMotorControl.spin_dir,gimbalControl.GimbalMotorControl.sniper_max_speed,gimbalControl.GimbalMotorControl.sniper_pos);
						 LK_MultiLoop_angleControl_limited(sadata,pitch_MAX_SPEED,LK_PITCH_HORIZON_ENCODE+gimbalControl.GimbalTargetInput.pitch_angle_d*800);//减速比1:8注意看
						 else  
						 LK_iqControl(sadata,_gimbalControl->GimbalMotorControl.pitch_target_output);
							//LK_iqControl(sadata,0);
					 }
					 else
						 LK_iqControl(sadata,0);
						 CANTransmit_U8(&hfdcan1, 0x141 , sadata);
	     break;
		}

	/*测距部分*/
	//extern uint8_t lastRobotState.sniper;

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
		//没灯了 板载ws2812灯自己写
	//有的兄弟，有的
	//WS2812SignalSend();
//	if(
//	WS2812_SPI_Ctrl(,,)
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
//系统辨识前进行扫频
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
			if(hfdcan1.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan3);
	}
 }
/**
  * @brief 测试用，用uart发送底盘电机数据
  * @note 发送数据为int16t，用uint8数组发送，一个数据占两字节，用memcpy复制内容，用DMA发送
  * @retval void
  */
  	
uint8_t debug_data[42];
//extern uint8_t stall_count;
extern DataFromJudge bulletSpeed;
extern float predict_speed0;
 float lastspeed=0;
// float temp=0;
// int temp1=0;
#include <stdio.h> 
#include "ekf_imu_solver.h"
 #include "shoot_speed_best_contrl.h"
//extern float temp_see[3];
//extern IMURecData imuRecData;
//extern float pitch_angle_from_match;
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
 extern ext_shoot_data_t ext_shoot_data;
 extern DJIGMotorRec chassisMotorRec[CHASSIS_MOTOR_NUM];
int16_t trans[6];
int cnt;
static void DebugTransmit(void)  //hxgdebug
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

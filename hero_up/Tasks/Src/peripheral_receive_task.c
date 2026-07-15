#include "tim.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "general_task_include.h"
#include "peripheral_transmit_task.h"
#include "CAN_driver.h"
#include "dt7_remote_driver.h"
#include "VT13_rc_ctrl.h"
#include "ekf_imu_solver.h"
#include "judge_receive.h"
#include "DMJ4310.h"
#include "LK_driver.h"
#include "dm_imu.h"
#include "usbd_cdc_if.h"
//#include "distance_check.h"
#include "LK_485_driver.h"
#include "../../PrivateDrivers/Board2Borad/Board2Board.h"
/*回传速度*/
extern float predict_speed0;
//串口接收数据缓冲区
uint8_t uart1RecBuffer[MAX_RECEIVE_BUFFER_LENGTH];
uint8_t uart5RecBuffer[MAX_RECEIVE_BUFFER_LENGTH];	
uint8_t uart6RecBuffer[MAX_RECEIVE_BUFFER_LENGTH];
uint8_t uart10RecBuffer[MAX_RECEIVE_BUFFER_LENGTH];
uint8_t uartServentRecBuffer[MAX_RECEIVE_BUFFER_LENGTH];
int64_t circle_angle;
volatile float shoot485_yaw_rx_d = 0.0f;
volatile uint8_t shoot485_yaw_rx_valid = 0;
volatile float servant485_pitch_d = 0.0f;   /* 下板传输的机体pitch角 */
volatile float servant485_roll_d  = 0.0f;   /* 下板传输的机体roll角  */
volatile float servant485_yaw_d   = 0.0f;   /* 下板传输的机体yaw角   */
volatile uint16_t servant485_current_hp = 0xFFFF;   /* 下板传输的裁判系统血量，默认最大值防止误触发 */
volatile uint8_t servant485_hp_zero_flag = 0;       /* HP归零保护标志，状态机消费后清零 */

//uint8_t uartPITCHRecBuffer[64];
/*--------------------------------------------------remote task region------------------------------------------------*/
static void NormRemoteCmdInit(NormRemoteCmd* norm_remote_cmd);
static void DT7ToNormCmd(NormRemoteCmd* norm_remote_cmd, const DT7CmdData* dt7_cmd_data);

/*遥操作接收相关变量*/
EventGroupHandle_t remoteRecEventGroup;		//remote_task数据来源相关事件组

DT7RecData dt7RecData;	//完成数据拼接及移位操作后的dt7指令
DT7CmdData dt7CmdData;	//完全解读后的dt7遥控器指令
const DT7CmdData* _dt7CmdData = &dt7CmdData;
uint8_t dt7RecBuffer[DT7_RC_FRAME_LEN];	//dt7原始数据接收区
uint8_t VT3RecBuffer[VT3_RC_FRAME_LEN];//VT3遥控器接收
VT13_RC_ctrl_t vt13_rc_ctrl_t;//VT3

NormRemoteCmd normRemoteCmd;	//将不同控制信号来源的信息整理归化到统一结构体中
const NormRemoteCmd* _normRemoteCmd = &normRemoteCmd;

 /**
 * @brief 遥操作指令接收任务，处理接收遥控器数据，自定义控制器等
 *新增超时时间
 */
 const TickType_t RCDelay = pdMS_TO_TICKS(500); //超时时间
void RemoteRecTask(void* argument)
{
	/*事件组初始化*/
	static EventBits_t currentEventGroupBits;	
	remoteRecEventGroup = xEventGroupCreate();	
	
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count;
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._remote_rec_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._remote_rec_task = &this_tick_count;
	
	current_tick_count = last_tick_count = xTaskGetTickCount();
	
	NormRemoteCmdInit(&normRemoteCmd);
	
	while(1)
	{
		currentEventGroupBits = xEventGroupWaitBits(remoteRecEventGroup,		/* 事件组的句柄 */
												   (EVENT_GROUP_BIT_ERROR|EVENT_GROUP_BIT_DT7|EVENT_GROUP_BIT_VT3),	/* 检测事件标志位 */
													pdTRUE,																	/* 满足添加时清除上面的事件位 */
													pdFALSE, 																/* 任意事件位被设置就会退出阻塞态 */
													RCDelay);																/*有超时 */
		
		task_counter++;
		
		/*根据当前接收到的信号数据来源作相应的处理：
		1.根据各自协议解码；
		2.将各自解码得到的控制指令抄到标准控制指令中*/
		if(currentEventGroupBits & EVENT_GROUP_BIT_DT7)
		{
			DT7RawDataUpdate(&dt7RecData, dt7RecBuffer);
			DT7DataProcess(&dt7CmdData, &dt7RecData);
			DT7ToNormCmd(&normRemoteCmd, &dt7CmdData);
		
		}
		else if(currentEventGroupBits & EVENT_GROUP_BIT_VT3){
			VT13_to_rc(VT3RecBuffer,&vt13_rc_ctrl_t);
		}
		else if(currentEventGroupBits & EVENT_GROUP_BIT_ERROR)//发生错误，但是这个功能没有被正常启动
		{
			NormRemoteCmdInit(&normRemoteCmd);			
			RemoteRecRestart();
		}else
    {
        // 如果函数返回了，但上面两个事件位都没有被置位，
        // 这就意味着等待超时了！
        // 这正是“拔掉接收机”后会发生的情况。
        // 在这里处理超时错误
        // 例如：重启接收逻辑、向上层报告通信中断等
        // 发生错误
        NormRemoteCmdInit(&normRemoteCmd);
        RemoteRecRestart();
    }
		
		
		/*计算任务实际运行周期*/
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
	}
}

/**
 * @brief 标准控制信息初始化，可用作清零
 */
static void NormRemoteCmdInit(NormRemoteCmd* norm_remote_cmd)
{
	memset(norm_remote_cmd, 0, sizeof(NormRemoteCmd));
	norm_remote_cmd->remote_source = ERROR_RECEIVE;//这里有写入
}

/**
 * @brief 遥控接收中断重新初始化
 */
void RemoteRecRestart(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
	__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}
 
/**
 * @brief DT7CmdData转为系统标准控制信息NormRemoteCmd
 */
static void DT7ToNormCmd(NormRemoteCmd* norm_remote_cmd, const DT7CmdData* dt7_cmd_data)
{
	norm_remote_cmd->remote_source = DT7;
	norm_remote_cmd->RelativeCH.ch0 = dt7_cmd_data->ch0 / DT7_RC_CH_MAX_RELATIVE;
	norm_remote_cmd->RelativeCH.ch1 = dt7_cmd_data->ch1 / DT7_RC_CH_MAX_RELATIVE;
	norm_remote_cmd->RelativeCH.ch2 = dt7_cmd_data->ch2 / DT7_RC_CH_MAX_RELATIVE;
	norm_remote_cmd->RelativeCH.ch3 = dt7_cmd_data->ch3 / DT7_RC_CH_MAX_RELATIVE;
	norm_remote_cmd->RelativeCH.ch4 = dt7_cmd_data->ch4 / DT7_RC_CH_MAX_RELATIVE;
	/*标准遥控器拨杆键值和DT7遥控器拨杆键值对应*/
	norm_remote_cmd->Switch.switch_L1 = dt7_cmd_data->switch_left;
	norm_remote_cmd->Switch.switch_R1 = dt7_cmd_data->switch_right;
	memcpy(&(norm_remote_cmd->PCKeyBoard), &(dt7_cmd_data->PCKeyBoard), sizeof(dt7_cmd_data->PCKeyBoard));
	norm_remote_cmd->PCMouse.mouse_speed_x = dt7_cmd_data->PCMouse.x;
	norm_remote_cmd->PCMouse.mouse_speed_y = dt7_cmd_data->PCMouse.y;
	norm_remote_cmd->PCMouse.mouse_speed_z = dt7_cmd_data->PCMouse.z;
	norm_remote_cmd->PCMouse.mouse_left = dt7_cmd_data->PCMouse.press_left;
	norm_remote_cmd->PCMouse.mouse_right = dt7_cmd_data->PCMouse.press_right;
}

/*---------------------------------------------------motor CAN receive region-------------------------------------------*/
/*使用CAN总线通信的的电机的接收结构体*/
DJIGMotorRec chassisMotorRec[CHASSIS_MOTOR_NUM];
const DJIGMotorRec *_chassisMotorRec = chassisMotorRec;

DJIGMotorRec fricMotorRec[FRIC_MOTOR_NUM];
const DJIGMotorRec *_fricMotorRec = fricMotorRec;

DJIGMotorRec pitchMotorRec;
const DJIGMotorRec* _pitchMotorRec = &pitchMotorRec;

DJIGMotorRec smallpitchMotorRec;//????怎么也用大疆结构体，妈的懒得改了
const DJIGMotorRec* _smallpitchMotorRec=&smallpitchMotorRec;

DMJ4310MotorRec stirMotorRec;
const DMJ4310MotorRec* _stirMotorRec = &stirMotorRec;
Pose externalRecPose;

SuperCapacity superCapacity;
const SuperCapacity* _superCapacity = &superCapacity;

extern GimbalControl gimbalControl;
extern const GimbalControl* _gimbalControl;
long can_heavy=0;
LkMotor_State_t lkmoto_State_t;
extern const LkMotor_State_t* _lkmoto_State_t;
/**
 * @brief fdcan1总线接收       //hxg
 */
float pitchrecangle;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	can_heavy++;
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  	{
		FDCAN_RxHeaderTypeDef RxHeader;
		uint8_t aData[8];
		HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &RxHeader, aData);
		switch(RxHeader.Identifier)
	{		
		default:
				break;
			case 0x141:
	
			if(aData[0]==0xA1||aData[0]==0xA6||aData[0]==0xA4){
			pitchMotorRec.frame_counter++;
			pitchMotorRec.mechanical_angle = aData[7] << 8 | aData[6];
			pitchMotorRec.mechanical_speed_rpm = aData[5] << 8 | aData[4]; 
			pitchMotorRec.torque_current_real = aData[3] << 8 | aData[2]; 
			pitchMotorRec.motor_temperature_d = aData[1];
			pitchrecangle=((float)pitchMotorRec.mechanical_angle-55791)/65536*360;//task0,看看水平角度编码器值
			}
			if(aData[0]==0x94)//读取单圈角度
			{
			circle_angle= (uint32_t)aData[4] |
										((uint32_t)aData[5] << 8) |
										((uint32_t)aData[6] << 16) |
										((uint32_t)aData[7] << 24);
			}
            if(aData[0]==0x92)//读取多圈角度
			{
			circle_angle= (int64_t)aData[1] |
										((int64_t)aData[2] << 8) |
										((int64_t)aData[3] << 16) |
										((int64_t)aData[4] << 24) |
                                        ((int64_t)aData[5] << 32) |
                                        ((int64_t)aData[6] << 40) |
                                        ((int64_t)aData[7] << 48) ;
            }
			break;
	}
	}
}
uint32_t multicircle;
float madiyo;
uint32_t bianmaqi=44000;
/**
 * @brief fdcan2 && fdcan3总线接收           //hxg
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  	{
		FDCAN_RxHeaderTypeDef RxHeader;
		uint8_t aData[8];
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, aData);
			if(hfdcan == & hfdcan2){
			switch(RxHeader.Identifier){
		case 0x201:
			
		case 0x202:
            
        case 0x203:
            fricMotorRec[RxHeader.Identifier-0x201].frame_counter++;
			fricMotorRec[RxHeader.Identifier-0x201].mechanical_angle = aData[0] << 8 | aData[1];
			fricMotorRec[RxHeader.Identifier-0x201].mechanical_speed_rpm = aData[2] << 8 | aData[3]; 
			fricMotorRec[RxHeader.Identifier-0x201].torque_current_real = aData[4] << 8 | aData[5]; 
			fricMotorRec[RxHeader.Identifier-0x201].motor_temperature_d = aData[6];
        break;
        case 0x204:
                               //888
        case 0x205:
        case 0x206:
        case 0x207:
             fricMotorRec[RxHeader.Identifier-0x202].frame_counter++;
			fricMotorRec[RxHeader.Identifier-0x202].mechanical_angle = aData[0] << 8 | aData[1];
			fricMotorRec[RxHeader.Identifier-0x202].mechanical_speed_rpm = aData[2] << 8 | aData[3]; 
			fricMotorRec[RxHeader.Identifier-0x202].torque_current_real = aData[4] << 8 | aData[5]; 
			fricMotorRec[RxHeader.Identifier-0x202].motor_temperature_d = aData[6];
        break;
            //hxg
			
		break;
			/*一点关于摩擦轮的思考：
			怎么说呢有大概20-30MS电流是拉满了，大胆猜测，过程给大没鸟用啊主要靠惯量动量交换
			简单验算一下：
			1.摩擦时间 在相似弹速下根据开环观测大概18MS 不超过25MS，
			2.PID满载电流时间 大概20-25MS
			3.做功时间....NMD懒得计算了*/
						
			
//				case 0x142:
//				if(aData[0]==0x94)
//				{
//				
//				multicircle = (uint32_t)aData[7] << 24 | aData[6] << 16 | aData[5] << 8 | aData[4];
//				smallpitchMotorRec.mechanical_angle=multicircle;//不要了，什么呀
//				pitchrecangle=(float)pitchMotorRec.mechanical_angle;
//				//gimbalControl.GimbalEstimate.small_pitch_actual_angle=(float)smallpitchMotorRec.mechanical_angle/100.f+(gimbalControl.GimbalEstimate.pitch_angle_d-(6266-(float)pitchMotorRec.mechanical_angle)/65535*360);
//				gimbalControl.GimbalEstimate.small_pitch_actual_angle=((float)multicircle-42497)/1000;
//				}
//				else if(aData[0]==0xA1)
//				{
//					smallpitchMotorRec.mechanical_speed_rpm = aData[5] << 8 | aData[4]; 
//					smallpitchMotorRec.torque_current_real = aData[3] << 8 | aData[2]; 
//					smallpitchMotorRec.motor_temperature_d = aData[1];
//				}
//				break;
			
				break;
//				case 0x211:
//					superCapacity.frame_counter++;
//					superCapacity.cap_volt = (float)(aData[0] | (aData[1] << 8))/100.0f;
//					superCapacity.power_limit = (float)(aData[2] | (aData[3] << 8));
//					superCapacity.real_power = (float)(aData[4] | (aData[5] << 8));
//					superCapacity.compensated_power = (float)(aData[6] | (aData[7] << 8));	
//				break;
			}
		}
		else if(hfdcan == &hfdcan3){
				B2BCanRxHandler(RxHeader.Identifier, aData);  /* B2B 0x220-0x222（CAN3专用于双板通信） */
			}
		}
	}
}

//int16_t fric_realspeed_left,fric_realspeed_right;
//int16_t fric_speed_left, fric_speed_right;
/*---------------------------------------------------------------------------imu task(if using onboard imu)-----------------------------------------------------------------------------------*/ 
/*板载运算--采用ekf计算姿态角*/
IMUUseEKFSolver imuUseEKFSolver;
const IMUUseEKFSolver* _onboardIMUUseEKF = &imuUseEKFSolver;
IMURecData imuRecData;
const IMURecData* _onboardIMURecForEKF = &imuRecData;

/*云台姿态记录*/
Pose gimbalPose;
const Pose* _gimbalPose = &gimbalPose;

/*外接imu数据*/

/**
  * @brief  imu温控
  * @param  none
  * @retval none
  * 
  */
#define OUT_STANDING_TEMPERATURE 40.0f
float KP=100.f;
float KI=100;
float KD=10.0;
#define MAX_OUT  500
float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;
static void OnboardIMUTemperatureControl(float real_temperature)
{
	      err_ll = err_l;
        err_l = err;
//				imu_data_rec.temperature_raw = (imu_data_rec.temperature - BMI088_TEMP_OFFSET) / BMI088_TEMP_FACTOR;
				err = OUT_STANDING_TEMPERATURE-imuRecData.temperature;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
				htim3.Instance->CCR4 = (uint16_t)out;
}

 
/**
 * @brief 更新姿态角结构体
 * @param IMUfromEKF IMUfromMahony
 * @note  根据当前使用的宏确定姿态角来源
 */

#define X 0
#define Y 1
#define Z 2

static void PoseUpdateFromIMU(Pose* pose, const IMUUseEKFSolver* imu_use_ekf)
{
	/*注意根据实际安装方向和自己取的正方向调整*/
	//规定pitch轴抬头为正，低头为负；yaw轴顺时针旋转为正，逆时针旋转为负
	#if defined ONBOARD_EKF_SOLVE
		
		pose->pitch_radps  =-imu_use_ekf->Gyro[X];
		pose->roll_radps   = imu_use_ekf->Gyro[Y];
		pose->yaw_radps    = -imu_use_ekf->Gyro[Z];	
	
		pose->pitch_d = -imu_use_ekf->Pitch_d;
		pose->yaw_d	  = -imu_use_ekf->Yaw_d;
		pose->roll_d  = imu_use_ekf->Roll_d;
		
	#else

	#endif
	GimbalPoseUpdate(pose->pitch_d, pose->pitch_radps, pose->yaw_d, pose->yaw_radps,pose->roll_d,pose->roll_radps);
}

/**
 * @brief 从imu获取姿态角任务
 */
void IMUTask(void* argument)
{
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count = 0;
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._imu_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._imu_task = &this_tick_count;
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	while(1)
	{
		/* 温控始终运行 */
		OnboardIMUTemperatureControl(imuRecData.temperature);
		
		/* 偏置已由 GetIMUOffset 写入实测值，直接进入EKF解算 */
		#if defined ONBOARD_EKF_SOLVE
			IMUSolverUseEKFUserFunc(&imuUseEKFSolver, &imuRecData);
		#endif
		PoseUpdateFromIMU(&gimbalPose, &imuUseEKFSolver);
		RS485_SendIMU();  /* IMU数据立即发出 */

		/* 通知 EstimateTask */
		{
			extern TaskHandle_t estimateTaskHandle;
			if (estimateTaskHandle != NULL)
				xTaskNotifyGive(estimateTaskHandle);
		}

		/* 控制链监控：记录 IMU 通知发出的时刻 */
		g_chain_timer.cyc_imu_notify = DWT->CYCCNT;

		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
		vTaskDelayUntil(&current_tick_count, IMU_TASK_PERIOD_SET);
	}
}

/*---------------------------------------------------------------------------upper computer communication-----------------------------------------------------------------------------------*/
UpperComputerComm upperComputerComm;
const UpperComputerComm* _upperComputerComm = &upperComputerComm;
extern uint8_t shit_last_PC_Receive_shoot_mode;
/**
 * @brief 算法上位机通信接收中断处理函数
 */
void UpperCommRecHandler(uint8_t* rec_buf, uint32_t size)
{
	if(UPPER_PC_COMM_REC_SOF == rec_buf[0] && UPPER_PC_COMM_REC_EOF == rec_buf[size - 1] && size == UPPER_PC_COMM_REC_LENGTH)
	{
		shit_last_PC_Receive_shoot_mode=upperComputerComm.Receive.shoot_mode;
		memcpy(&upperComputerComm.Receive, rec_buf, sizeof(upperComputerComm.Receive));
		DoubleEdgeLimiter(upperComputerComm.Receive.target_pitch_angle_d,0,30);
		
		DoubleEdgeLimiter(upperComputerComm.Receive.target_yaw_angle_d,-10,10);
		upperComputerComm.rec_counter++;
	}
}

/// @brief 与算法上位机通信任务
/// @param argument 
void UpperPCCommTask(void* argument)
{
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count = 0;
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._upper_pc_comm_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._upper_pc_comm_task = &this_tick_count;
	
	upperComputerComm.Send.sof = UPPER_PC_COMM_REC_SOF;
	upperComputerComm.Send.eof = UPPER_PC_COMM_REC_EOF;
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	
	while(1)
	{
		static uint16_t no_rec_counter = 0;
		
		task_counter++;
		
		//长时间无接收，重置
		static uint16_t last_rec_counter = 0;
		if(upperComputerComm.rec_counter == last_rec_counter)
		{
			no_rec_counter++;
			//该操作针对不支持USB热拔插的上位机，在算法重启程序后无法正确使用USB port造成的一系列问题的解决办法--重启外设
			//这里每隔一定时间重启外设USB，主要是为了防止算法程序死掉或意外断联后无法接收到正确的数据
			//CAUTION:在MCU被烧写程序期间，USB口处于不定态，此时禁止算法重新启动代码open USB port，否则将无法检测到正确的USB port，发生此情况只能断电重启
			if(no_rec_counter > 500)//重启间隔时间不能太长（NX非重启二次运行程序建立通信的必要条件是MCU重新初始化过一次USB口），也不能太短（外设频繁初始化导致linux端检测到USB port的窗口变小）
				no_rec_counter = 0;
			else if(no_rec_counter > 20)
			{
				//memset(&upperComputerComm.Receive, 0, UPPER_PC_COMM_REC_LENGTH);
			}
		}
		else
		{
			no_rec_counter = 0;
			last_rec_counter = upperComputerComm.rec_counter;
		}
			
		if(ext_game_robot_status.robot_id < 50)
			upperComputerComm.Send.self_team = RED_TEAM_FRAME;
		else
			upperComputerComm.Send.self_team = BLUE_TEAM_FRAME;
				
//		if(_robotState->aim_mode == NORMAL_MODE)//暂时改动用来测自瞄
		if(_robotState->sniper==SNIPER_OFF)
			upperComputerComm.Send.task_mode = 0;
		else 
			upperComputerComm.Send.task_mode = 2;
		
//		if(_normRemoteCmd->PCKeyBoard.level_key_C)
//		{
//			upperComputerComm.Send.task_mode = 3;
//		}
		
		//upperComputerComm.Send.task_mode=upperComputerComm.Send.task_mode;
		
		upperComputerComm.Send.bullet_speed = ext_shoot_data.initial_speed;
		upperComputerComm.Send.gimbal_pitch_d = _gimbalControl->GimbalEstimate.pitch_angle_d;
		upperComputerComm.Send.gimbal_yaw_d = shoot485_yaw_rx_d;
		upperComputerComm.Send.gimbal_yaw_dps = _gimbalControl->GimbalEstimate.yaw_angular_velocity_dps;
		upperComputerComm.Send.cam_target = _robotState->cam_target;
		
		#ifdef UPPER_PC_TRANSMIT_ENABLE
		//HAL_UART_Transmit_DMA(&MINIPC_UART, (uint8_t*)&(upperComputerComm.Send), UPPER_PC_COMM_SEND_LENGTH);
		#endif
		CDC_Transmit_HS((uint8_t*)&(upperComputerComm.Send), UPPER_PC_COMM_SEND_LENGTH);//使用虚拟串口CDC的库，类似uart但是其信号层不一样

		/* B2B CAN: GimbalTarget 100Hz -> hero_down */
		{
			float ty = _upperComputerComm->Receive.target_yaw_angle_d;
			float tp = _upperComputerComm->Receive.target_pitch_angle_d;
			if (worldGimbal.enable) {
				ty = worldGimbal.WorldGimbalControl.q_yaw_cmd_deg;
				tp = worldGimbal.WorldGimbalControl.q_pitch_cmd_deg;
			}
			B2BSendGimbalTarget(ty, tp);
		}
		//memset(upperComputerComm.Send.reserved, 0, 3);  // 发送后清零，由state_task下一周期置位
		/*计算任务实际运行周期*/
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;	

		vTaskDelayUntil(&current_tick_count, UPPER_COMM_TASK_PERIOD_SET);

	}
}
/*---------------------------------------------------------function for handle judge data-------------------------------------------------------------------------------*/
//shoot485_yaw_rx_d
DataFromJudge bulletSpeed;
const DataFromJudge* _bulletSpeed = &bulletSpeed;
//float predict_speed_array[40]={0};
//uint8_t speedindex=0;
/// @brief 弹速统计处理
/// @param shooter_30_state 
void BulletSpeedStatistics(uint8_t shooter_30_state)
{
	
    // 弹速统计 - 全精度版本
    static float speed[30];           // 直接存储浮点数，避免精度损失
    static uint8_t freq[110] = {0};   // 记录15.40-16.49的频率（0.01间隔）
    uint8_t sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0, sum5 = 0;
    float median = 0.0f, mode = 0.0f, average = 0.0f;
    uint8_t count = (shooter_30_state) ? 30 : bulletSpeed.record_shoot_count;

    // 边界检查
    if (count <= 0) return;

    // 清空频率数组
    memset(freq, 0, sizeof(freq));

    // 复制弹速数据并排序
    memcpy(speed, bulletSpeed.bullet_speed, count * sizeof(float));

    // 使用插入排序（稳定且适合小规模数据）
    for (int i = 1; i < count; i++) {
        float key = speed[i];
        int j = i - 1;
        while (j >= 0 && speed[j] > key) {
            speed[j + 1] = speed[j];
            j--;
        }
        speed[j + 1] = key;
    }

    // 频率统计（0.01间隔）
    for (int i = 0; i < count; i++) {
        if (speed[i] >= 15.40f && speed[i] < 16.50f) {
            int index = (int)((speed[i] - 15.40f) * 100);  // 映射到0-109
            freq[index]++;
        }
    }

    // 计算中位数
    if (count % 2 == 0) {
        median = (speed[count / 2 - 1] + speed[count / 2]) / 2.0f;
    }
    else {
        median = speed[count / 2];
    }
		bulletSpeed.median=median;
    // 计算平均数
    for (int i = 0; i < count; i++) {
        average += speed[i];
    }
    average /= count;
		bulletSpeed.average= average;

    // 计算众数（允许±0.01的容差）
    uint8_t maxCount = 0;
    for (int i = 0; i < 110; i++) {
        // 计算当前区间及其相邻区间的总频率
        uint8_t currentFreq = freq[i];
        if (i > 0) currentFreq += freq[i - 1];
        if (i < 109) currentFreq += freq[i + 1];

        if (currentFreq > maxCount) {
            maxCount = currentFreq;
            mode = 15.40f + (float)i / 100.0f;  // 区间中点值
					
        }
    }
       bulletSpeed.mode=mode;
    // 统计各个速度指标附近的弹数（±0.01范围）
    for (int i = 0; i < count; i++) {
        // 中位数附近
        if (fabsf(speed[i] - median) <= 0.01f) {
            sum2++;
            if (fabsf(speed[i] - median) < 0.001f) sum3++;  // 严格相等
        }

        // 众数附近（±0.01范围）
        if (fabsf(speed[i] - mode) <= 0.01f) {
            sum1++;
        }

        // 平均数附近
        if (fabsf(speed[i] - average) <= 0.01f) {
            sum4++;
            if (fabsf(speed[i] - average) < 0.001f) sum5++;
        }
    }

    // 计算命中率和精度指标
    bulletSpeed.hit_rate_median = (float)sum2 / count;
    bulletSpeed.relative_accuracy_median = (float)sum3 / sum2;
    bulletSpeed.absolute_accuracy_median = (float)sum3 / count;

    bulletSpeed.hit_rate_mode = (float)sum1 / count;
    bulletSpeed.relative_accuracy_mode = (float)maxCount / sum1;
    bulletSpeed.absolute_accuracy_mode = (float)maxCount / count;

    bulletSpeed.hit_rate_average = (float)sum4 / count;
    bulletSpeed.relative_accuracy_average = (float)sum5 / sum4;
    bulletSpeed.absolute_accuracy_average = (float)sum5 / count;

    // 选择命中率最高的指标作为预测速度
    float max_hit_rate = fmaxf(bulletSpeed.hit_rate_median,
        fmaxf(bulletSpeed.hit_rate_mode, bulletSpeed.hit_rate_average));

    bulletSpeed.predict_speed =
        (max_hit_rate == bulletSpeed.hit_rate_median)? median :
        (max_hit_rate == bulletSpeed.hit_rate_mode)? mode :
        average;
		
//		if(speedindex<40){
//		predict_speed_array[speedindex]=bulletSpeed.predict_speed;
//			speedindex++;
//		}
//		else
//			speedindex=0;
}

/// @brief 弹速读入函数，当裁判系统发射数据帧更新时被调用，接口写在judge_receive中
/// @param  
void BulletSpeedReceive(void)
{
	static uint8_t shooted_30_flag = 0;   //为1标志已经打了30发
	if(ext_shoot_data.launching_frequency != 0)
	{
		bulletSpeed.bullet_speed[bulletSpeed.record_shoot_count++] = ext_shoot_data.initial_speed;
		BulletSpeedStatistics(shooted_30_flag);
		
		if(bulletSpeed.record_shoot_count == 30)
		{
			bulletSpeed.record_shoot_count = 1;
			shooted_30_flag = 1;
		}
		ext_shoot_data.launching_frequency = 0;
	}
}
/*--------------------------------------------------------------------------utility------------------------------------------------------------------------------------*/
/**
 * @brief 外设接收初始化
 */

void PeripheralRecEnable(void)
{
	/*DT7 remote*/
	DT7RemoteRecEnable(&RC_UART, uart5RecBuffer);
	
	//一键init所有can总线
	can_bsp_init();
	B2BInit();
	
	/*UpperCommRec init*/
	HAL_UARTEx_ReceiveToIdle_DMA(&MINIPC_UART, uart1RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
	
	/*onboard imu init*/
	#if defined ONBOARD_EKF_SOLVE
		IMUSolverUseEKFInitialize(&imuUseEKFSolver, &imuRecData, IMU_TASK_PERIOD_SET / 1000.0f);
	#endif
	
	/*judge receive init*/
	HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
	__HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&LASER_UART, uart10RecBuffer,LASER_UART_LENGTH);
	__HAL_DMA_DISABLE_IT(LASER_UART.hdmarx, DMA_IT_HT);
	
	/*双板通讯*/
}   
/**
 * @brief 串口dma不定长接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
			/*激光串口*/
	    if (huart == &LASER_UART){

        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    }
	/*遥控器串口*/
	if(huart == &RC_UART)
	{		
		if(DT7_RC_FRAME_LEN == Size)
		{
			memcpy(dt7RecBuffer, uart5RecBuffer, Size);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;		
			xEventGroupSetBitsFromISR(remoteRecEventGroup, EVENT_GROUP_BIT_DT7, &xHigherPriorityTaskWoken);
		}
		if(Size==VT3_RC_FRAME_LEN){
			memcpy(VT3RecBuffer, uart5RecBuffer, Size);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;		
			xEventGroupSetBitsFromISR(remoteRecEventGroup, EVENT_GROUP_BIT_VT3, &xHigherPriorityTaskWoken);
		}
		memset(uart5RecBuffer,0, MAX_RECEIVE_BUFFER_LENGTH);
		HAL_UARTEx_ReceiveToIdle_DMA(&RC_UART, uart5RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
		__HAL_DMA_DISABLE_IT(RC_UART.hdmarx, DMA_IT_HT);		
	}
	/*算法串口*/
	if(huart == &MINIPC_UART)
	{
		uint8_t temp_buffer[Size];
		memcpy(temp_buffer, uart1RecBuffer, Size);
		memset(uart1RecBuffer,0, MAX_RECEIVE_BUFFER_LENGTH);
		UpperCommRecHandler(temp_buffer, Size);
		static uint8_t error_count;
		while(HAL_UARTEx_ReceiveToIdle_DMA(&MINIPC_UART, uart1RecBuffer, MAX_RECEIVE_BUFFER_LENGTH) == HAL_BUSY)
		{

			error_count ++;	
			__HAL_UNLOCK(&MINIPC_UART);
			if(error_count >= 4)
				break;
		}
		error_count = 0;
	}
	/*裁判串口*/
	if(huart == &REFEREE_UART)
	{
		uint8_t temp_buffer[Size];
		memcpy(temp_buffer, uart6RecBuffer, Size);
		memset(uart6RecBuffer,0, MAX_RECEIVE_BUFFER_LENGTH);
		RefereeReceive(Size, temp_buffer);
		
		HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
		__HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);			
	}
	if(huart == &SERVANT_485_UART){
		//遥控器数据处理
		if(uartServentRecBuffer[0]==0x07){
			//18U数据
			memcpy(dt7RecBuffer, uartServentRecBuffer+1, 18U);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;		
			xEventGroupSetBitsFromISR(remoteRecEventGroup, EVENT_GROUP_BIT_DT7, &xHigherPriorityTaskWoken);
		}
		else if(uartServentRecBuffer[0]==0x03){
			//21U数据
			memcpy(VT3RecBuffer, uartServentRecBuffer+1, 21U);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;		
			xEventGroupSetBitsFromISR(remoteRecEventGroup, EVENT_GROUP_BIT_VT3, &xHigherPriorityTaskWoken);
		}
		else if(uartServentRecBuffer[0]==0x88){
			//无遥控器数据

		}
		/* 固定偏移解析（与发送端double_mcu_frame布局一致）：
		   [22..25] yaw_enc, [26..27] HP, [28..31] pitch, [32..35] roll, [36..39] yaw */
		if(Size >= 40U){
			memcpy((void*)&shoot485_yaw_rx_d, &uartServentRecBuffer[22], sizeof(float));
			shoot485_yaw_rx_valid = 1;
			/* 裁判系统血量 */
			uint16_t hp;
			memcpy((void*)&hp, &uartServentRecBuffer[26], sizeof(uint16_t));
			servant485_current_hp = hp;
			if(hp == 0){
				servant485_hp_zero_flag = 1;
			}
			/* 机体姿态角 */
			memcpy((void*)&servant485_pitch_d, &uartServentRecBuffer[28], sizeof(float));
			memcpy((void*)&servant485_roll_d,  &uartServentRecBuffer[32], sizeof(float));
			memcpy((void*)&servant485_yaw_d,   &uartServentRecBuffer[36], sizeof(float));
		}
	  HAL_UARTEx_ReceiveToIdle_DMA(&SERVANT_485_UART, uartServentRecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
	  __HAL_DMA_DISABLE_IT(SERVANT_485_UART.hdmarx, DMA_IT_HT);
	}
	if(huart == &SHOOT_485_UART){
			}
}
extern QueueHandle_t g_musicQueue;

/*上电时，瞬间出现异常uart帧，可能导致uart错误中断，uart失能，因此复写hal库中的函数*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	//MX_USART10_UART_Init();直接初始化，不行，会循环
	 __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
	  
	HAL_UART_AbortReceive(huart);//使用这个更加健壮
  //CLEAR_BIT(((DMA_Stream_TypeDef*)(huart1.hdmarx)->Instance)->CR, DMA_IT_HT);//怎么Uart1也有问题
	//使用 __HAL_UART_CLEAR_FLAG 清除ORE、NE、FE、PE标志，确保硬件状态复位
	if(huart==&huart5)
		NormRemoteCmdInit(&normRemoteCmd);	
	//DBUS的保护没有
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer,MAX_RECEIVE_BUFFER_LENGTH);
//	HAL_UARTEx_ReceiveToIdle_DMA(&MINIPC_UART, uart1RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
//	HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
//	HAL_UARTEx_ReceiveToIdle_DMA(&DEBUG_UART, uart6RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
	if (huart->Instance == USART10)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
		__HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
	}
}
	#define FDCAN_ERROR_MASK (FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA)
/*没有24V供电启动CAN会导致CAN错误失能，然后再加上24V也启动不了，这里试图让CAN重新启动*/
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan){
	extern CANTxMonitor canMonitor[3];
	uint8_t idx = 0;
	if (hfdcan == &hfdcan2) idx = 1;
	else if (hfdcan == &hfdcan3) idx = 2;
	canMonitor[idx].err_callback++;

	/*清除错误位*/
    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_ERROR_MASK);

	/* 每 1000 次错误做一次完整重启 */
	if (canMonitor[idx].err_callback % 1000 == 0) {
		canMonitor[idx].err_reinit++;
		HAL_FDCAN_DeInit(hfdcan);
		HAL_FDCAN_Init(hfdcan);
		can_filter_init();           // 重新配置滤波器
		HAL_FDCAN_Start(hfdcan);     // 重启 CAN
		/* 重新使能接收中断 */
		if (hfdcan == &hfdcan1) {
			uint32_t target_interrupts = 0x00038001;
			HAL_FDCAN_ActivateNotification(hfdcan, target_interrupts, NULL);
		} else if (hfdcan == &hfdcan2) {
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_TX_EVT_FIFO_NEW_DATA, NULL);
		} else {
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, NULL);
		}
	}
}

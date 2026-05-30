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
#include "distance_measure.h"
//#include "distance_check.h"
//关节电机lf rf lb rb 1 2 3 4
// master 11 12 13 14
//履带l r 5 6
//master 15 16
//yaw master 17
//7 
//记得拨盘改master 18 8
//can2 3508*4+履带
//can1 joint*4+yaw*1+stir
//can3 超电
#include "LK_485_driver.h"
/*回传速度*/
extern float predict_speed0;
//串口接收数据缓冲区
uint8_t uart1RecBuffer[32]; /* 上位机(USB CDC) 当前未启用, 32B占位 */
uint8_t uart5RecBuffer[64];	/* 遥控器: DT7=18B / VT3=21B, 64B足够 */
uint8_t uart6RecBuffer[160]; /* 裁判系统 最大帧~128B, 留160B余量 */
uint8_t uart10RecBuffer[49]; 
int64_t circle_angle;
uint8_t uartServentRecBuffer[36]; /* RS485云台→底盘 32B协议帧，预留余量 */
uint8_t uartMasterRecBuffer[64]; /* RS485底盘→云台 MCU_FRAME_LEN=22B */
uint8_t distance_buffer[49];

/* 云台板下发的yaw/pitch角度和角速度（通过RS485接收） */
volatile float gimbal_yaw_rx_d = 0.0f;
volatile float gimbal_yaw_dps_rx = 0.0f;
volatile float gimbal_yaw_target_rx_d = 0.0f;  /* 上位机目标yaw */
volatile float gimbal_pitch_target_rx_d = 0.0f; /* 新增：上位机目标pitch */
volatile float gimbal_pitch_rx_d = 0.0f;       /* 云台板下发的实际pitch角度 */
volatile int16_t gimbal_fric_rpm_rx = 0;
volatile int16_t gimbal_fric_rpm_rx_arr[6] = {0};
volatile uint8_t gimbal_yaw_rx_valid = 0;

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
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5RecBuffer, sizeof(uart5RecBuffer));
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

DJIGMotorRec yawMotorRec;
const DJIGMotorRec* _yawMotorRec = &yawMotorRec;

DMJ4310MotorRec DMyawMotorRec;
const DMJ4310MotorRec* _DMyawMotorRec = &DMyawMotorRec;
	
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

//lf rf lb rb 1 2 3 4
// master 11 12 13 14
//l r 5 6
//master 15 16
DMJ4310MotorRec jointMotorEec[4];
const DMJ4310MotorRec* _jointMotorEec = jointMotorEec;

DMJ4310MotorRec caterpillarMotorRec[2];
const DMJ4310MotorRec* _caterpillarMotorRec = caterpillarMotorRec;

extern GimbalControl gimbalControl;
extern const GimbalControl* _gimbalControl;
long can_heavy=0;
LkMotor_State_t lkmoto_State_t;
extern const LkMotor_State_t* _lkmoto_State_t;
/**
 * @brief fdcan1总线接收       //hxg
 */
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
		case 0x017:
			DMyawMotorRec.frame_counter++;
	    DMyawMotorRec.id = (aData[0])&0x0F;
			DMyawMotorRec.state = (aData[0])>>4;
			DMyawMotorRec.p_int=(aData[1]<<8)|aData[2];    
			DMyawMotorRec.v_int=(aData[3]<<4)|(aData[4]>>4);
			DMyawMotorRec.t_int=((aData[4]&0xF)<<8)|aData[5];
			DMyawMotorRec.pos_d = -uint_to_float(DMyawMotorRec.p_int, -(DM_YAW_MAX_ENCODE_D), +(DM_YAW_MAX_ENCODE_D),16);
			DMyawMotorRec.vel_radps = uint_to_float(DMyawMotorRec.v_int, -45.0, 45.0, 12);
			DMyawMotorRec.toq = uint_to_float(DMyawMotorRec.t_int, -     12.0, 12.0, 12);
			DMyawMotorRec.Tmos = (float)(aData[6]);
			DMyawMotorRec.Tcoil = (float)(aData[7]);
		break;
		

	case 0x011:
	case 0x012:
	case 0x013:
	case 0x014:
				jointMotorEec[RxHeader.Identifier-0x011].frame_counter++;
				jointMotorEec[RxHeader.Identifier-0x011].id = (aData[0])&0x0F;
				jointMotorEec[RxHeader.Identifier-0x011].state = (aData[0])>>4;
				jointMotorEec[RxHeader.Identifier-0x011].p_int=(aData[1]<<8)|aData[2];         //接收到的原始数据
				jointMotorEec[RxHeader.Identifier-0x011].v_int=(aData[3]<<4)|(aData[4]>>4);
				jointMotorEec[RxHeader.Identifier-0x011].t_int=((aData[4]&0xF)<<8)|aData[5];
				jointMotorEec[RxHeader.Identifier-0x011].pos_d = uint_to_float(jointMotorEec[RxHeader.Identifier-0x011].p_int, -(DM_YAW_MAX_ENCODE_D), +(DM_YAW_MAX_ENCODE_D),16);
				jointMotorEec[RxHeader.Identifier-0x011].vel_radps = uint_to_float(jointMotorEec[RxHeader.Identifier-0x011].v_int, -10.0, 10.0, 12);
				jointMotorEec[RxHeader.Identifier-0x011].toq = uint_to_float(jointMotorEec[RxHeader.Identifier-0x011].t_int, -28.0, 28.0, 12);
				jointMotorEec[RxHeader.Identifier-0x011].Tmos = (float)(aData[6]);
				jointMotorEec[RxHeader.Identifier-0x011].Tcoil = (float)(aData[7]);
		break;

	case 0x018:
			stirMotorRec.frame_counter++;
	    stirMotorRec.id = (aData[0])&0x0F;
			stirMotorRec.state = (aData[0])>>4;
			stirMotorRec.p_int=(aData[1]<<8)|aData[2];         //接收到的原始数据
			stirMotorRec.v_int=(aData[3]<<4)|(aData[4]>>4);
			stirMotorRec.t_int=((aData[4]&0xF)<<8)|aData[5];
		  //对原始数据进行类型转换
			stirMotorRec.pos_d = uint_to_float(stirMotorRec.p_int, -(DM_MOTO_MAX_ENCODE_D), +(DM_MOTO_MAX_ENCODE_D),16);
			/*这里准备要改角度和编码器的映射，公式是
			100发弹丸--65535
			(100*60)度--65535*/
			//stirMotorRec.pos_d= uint_to_float(stirMotorRec.p_int, -180, 180, 16);
			stirMotorRec.vel_radps = uint_to_float(stirMotorRec.v_int, -30.0, 30.0, 12);
			stirMotorRec.toq = uint_to_float(stirMotorRec.t_int, -10.0, 10.0, 12);
			stirMotorRec.Tmos = (float)(aData[6]);
			stirMotorRec.Tcoil = (float)(aData[7]);
		break;
	}
	}
}
uint32_t multicircle;
float madiyo;
float pitchrecangle;
uint32_t bianmaqi=44000;
////can过滤器,lk握手式发送,注意总线在哪里,达秒电机使能
/**
 * @brief fdcan2 && fdcan3总线接收           //hxg
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
  	{
		FDCAN_RxHeaderTypeDef RxHeader;
		uint8_t aData[8];
		HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1, &RxHeader, aData);
		if(hfdcan == & hfdcan2){
		switch(RxHeader.Identifier){
				case 0x201:
				case 0x202:
				case 0x203:
				case 0x204:
							chassisMotorRec[RxHeader.Identifier-0x201].frame_counter++;
							chassisMotorRec[RxHeader.Identifier-0x201].mechanical_angle = aData[0] << 8 | aData[1];
							chassisMotorRec[RxHeader.Identifier-0x201].mechanical_speed_rpm = aData[2] << 8 | aData[3]; 
							chassisMotorRec[RxHeader.Identifier-0x201].torque_current_real = aData[4] << 8 | aData[5]; //-16380-16380 
							chassisMotorRec[RxHeader.Identifier-0x201].motor_temperature_d = aData[6];
						
		    	break;	
				case 0x015:
				case 0x016:
							caterpillarMotorRec[RxHeader.Identifier-0x015].frame_counter++;
							caterpillarMotorRec[RxHeader.Identifier-0x015].id = (aData[0])&0x0F;
							caterpillarMotorRec[RxHeader.Identifier-0x015].state = (aData[0])>>4;
							caterpillarMotorRec[RxHeader.Identifier-0x015].p_int=(aData[1]<<8)|aData[2];       
							caterpillarMotorRec[RxHeader.Identifier-0x015].v_int=(aData[3]<<4)|(aData[4]>>4);
							caterpillarMotorRec[RxHeader.Identifier-0x015].t_int=((aData[4]&0xF)<<8)|aData[5];
							caterpillarMotorRec[RxHeader.Identifier-0x015].pos_d = uint_to_float(caterpillarMotorRec[RxHeader.Identifier-0x015].p_int, -(DM_MOTO_MAX_ENCODE_D), +(DM_MOTO_MAX_ENCODE_D),16);
							caterpillarMotorRec[RxHeader.Identifier-0x015].vel_radps = uint_to_float(caterpillarMotorRec[RxHeader.Identifier-0x015].v_int, -45.0, 45.0, 12);
							caterpillarMotorRec[RxHeader.Identifier-0x015].toq = uint_to_float(caterpillarMotorRec[RxHeader.Identifier-0x015].t_int, -18.0, 18.0, 12);
							caterpillarMotorRec[RxHeader.Identifier-0x015].Tmos = (float)(aData[6]);
							caterpillarMotorRec[RxHeader.Identifier-0x015].Tcoil = (float)(aData[7]);
				break;

					
                case 0x211://英雄电容控制板
		 			    superCapacity.frame_counter++;
		 					superCapacity.cap_volt = (float)(aData[0] | (aData[1] << 8))/100.0f;
		 					superCapacity.power_limit = (float)(aData[2] | (aData[3] << 8));
		 					superCapacity.real_power = (float)(aData[4] | (aData[5] << 8));
		 					superCapacity.compensated_power = (float)(aData[6] | (aData[7] << 8));	
			
		        break;
			    default:
				break;
		}
		}
		else if(hfdcan == &hfdcan3){
			switch(RxHeader.Identifier){
	
                case 0x211://英雄电容控制板
							superCapacity.frame_counter++;
							superCapacity.cap_volt = (float)(aData[0] | (aData[1] << 8))/100.0f;
							superCapacity.power_limit = (float)(aData[2] | (aData[3] << 8));
							superCapacity.real_power = (float)(aData[4] | (aData[5] << 8));
							superCapacity.compensated_power = (float)(aData[6] | (aData[7] << 8));	
					break;
				default:
					break;
			}
		}
	}
}

/*---------------------------------------------------------------------------imu task(if using onboard imu)-----------------------------------------------------------------------------------*/ 
/*板载运算--采用ekf计算姿态角*/
IMUUseEKFSolver imuUseEKFSolver;
const IMUUseEKFSolver* _onboardIMUUseEKF = &imuUseEKFSolver;
IMURecData imuRecData;
const IMURecData* _onboardIMURecForEKF = &imuRecData;

/*云台姿态记录*/
Pose gimbalPose;
const Pose* _gimbalPose = &gimbalPose;

/* IMU陀螺仪零偏校准全局变量（方便调试观察） */
#define GYRO_BIAS_CALIB_TARGET_TEMP  39.0f
#define GYRO_BIAS_CALIB_TOTAL_SAMPLE 5000
#define GYRO_BIAS_CALIB_DECIMATE     10       /* 每2个任务周期采集1次 */

float    g_gyro_bias_sum[3] = {0.0f, 0.0f, 0.0f};
float    g_gyro_bias_result[3] = {0.0f, 0.0f, 0.0f};
uint16_t g_gyro_bias_sample_cnt = 0;
uint8_t  g_gyro_bias_state = 0;               /* 0=等待温度达标, 1=采集中, 2=校准完成 */
uint8_t  g_gyro_bias_decimate_cnt = 0;

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

		/* 绝对系运动加速度（已低通滤波、已去重力，后续由力控模块旋转到机体系） */
		pose->accel_x = imu_use_ekf->MotionAccel_n[X];
		pose->accel_y = imu_use_ekf->MotionAccel_n[Y];
		pose->accel_z = imu_use_ekf->MotionAccel_n[Z];
		
	#else

	#endif
	GimbalPoseUpdate(pose->pitch_d, pose->pitch_radps, pose->yaw_d, pose->yaw_radps,pose->roll_d,pose->roll_radps);
}

/**
 * @brief IMU陀螺仪零偏校准，温度达到39°C后开始采集5000个数据，每2个任务周期采1次，取平均写入gyro_offset
 * @param[in,out] imu_data_rec  IMU接收数据结构体指针（读取gyro_raw，写入gyro_offset）
 * @note  全局变量g_gyro_bias_*保留最终值便于调试；校准完成后state=2不再重复执行
 */
static void GetIMUGyroBiasCalibration(IMURecData* imu_data_rec)
{
	/* 已校准完成，不再重复 */
	if (g_gyro_bias_state == 2)
	{
		return;
	}

	/* 阶段0：等待IMU温度达到39°C */
	if (g_gyro_bias_state == 0)
	{
		if (imu_data_rec->temperature >= GYRO_BIAS_CALIB_TARGET_TEMP)
		{
			g_gyro_bias_state = 1;
			g_gyro_bias_sample_cnt = 0;
			g_gyro_bias_decimate_cnt = 0;
			g_gyro_bias_sum[0] = 0.0f;
			g_gyro_bias_sum[1] = 0.0f;
			g_gyro_bias_sum[2] = 0.0f;
		}
		return;
	}

	/* 阶段1：采集中，每GYRO_BIAS_CALIB_DECIMATE个周期采集1次 */
	g_gyro_bias_decimate_cnt++;
	if (g_gyro_bias_decimate_cnt < GYRO_BIAS_CALIB_DECIMATE)
	{
		return;
	}
	g_gyro_bias_decimate_cnt = 0;

	/* 累加原始角速度（rad/s，尚未减去偏置）：gyro + gyro_offset 还原为未减偏置的物理值 */
	g_gyro_bias_sum[0] += imu_data_rec->gyro[0] + imu_data_rec->gyro_offset[0];
	g_gyro_bias_sum[1] += imu_data_rec->gyro[1] + imu_data_rec->gyro_offset[1];
	g_gyro_bias_sum[2] += imu_data_rec->gyro[2] + imu_data_rec->gyro_offset[2];
	g_gyro_bias_sample_cnt++;

	/* 采集满5000个数据后计算平均值并写入gyro_offset */
	if (g_gyro_bias_sample_cnt >= GYRO_BIAS_CALIB_TOTAL_SAMPLE)
	{
		g_gyro_bias_result[0] = g_gyro_bias_sum[0] / (float)GYRO_BIAS_CALIB_TOTAL_SAMPLE;
		g_gyro_bias_result[1] = g_gyro_bias_sum[1] / (float)GYRO_BIAS_CALIB_TOTAL_SAMPLE;
		g_gyro_bias_result[2] = g_gyro_bias_sum[2] / (float)GYRO_BIAS_CALIB_TOTAL_SAMPLE;

		imu_data_rec->gyro_offset[0] = g_gyro_bias_result[0];
		imu_data_rec->gyro_offset[1] = g_gyro_bias_result[1];
		imu_data_rec->gyro_offset[2] = g_gyro_bias_result[2];

		g_gyro_bias_state = 2;
	}
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
		/*任务主进程*/
		#if defined ONBOARD_EKF_SOLVE
			IMUSolverUseEKFUserFunc(&imuUseEKFSolver, &imuRecData);
		#endif

		OnboardIMUTemperatureControl(imuRecData.temperature);		
		
		PoseUpdateFromIMU(&gimbalPose, &imuUseEKFSolver);
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
		upperComputerComm.Send.gimbal_yaw_d = _gimbalControl->GimbalEstimate.yaw_angle_d;
		upperComputerComm.Send.gimbal_yaw_dps = _gimbalControl->GimbalEstimate.yaw_angular_velocity_dps;
		
		#ifdef UPPER_PC_TRANSMIT_ENABLE
		//HAL_UART_Transmit_DMA(&MINIPC_UART, (uint8_t*)&(upperComputerComm.Send), UPPER_PC_COMM_SEND_LENGTH);
		#endif
		CDC_Transmit_HS((uint8_t*)&(upperComputerComm.Send), UPPER_PC_COMM_SEND_LENGTH);//使用虚拟串口CDC的库，类似uart但是其信号层不一样
		/*计算任务实际运行周期*/
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;		

		vTaskDelayUntil(&current_tick_count, UPPER_COMM_TASK_PERIOD_SET);

	}
}
/*---------------------------------------------------------function for handle judge data-------------------------------------------------------------------------------*/
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
	
	/*UpperCommRec init*/
	//HAL_UARTEx_ReceiveToIdle_DMA(&MINIPC_UART, uart1RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
	
	/*onboard imu init*/
	#if defined ONBOARD_EKF_SOLVE
		IMUSolverUseEKFInitialize(&imuUseEKFSolver, &imuRecData, IMU_TASK_PERIOD_SET / 1000.0f);
	#endif
	
	/*judge receive init*/
	HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, sizeof(uart6RecBuffer));
	__HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&LASER_UART, uart10RecBuffer,LASER_UART_LENGTH);
	__HAL_DMA_DISABLE_IT(LASER_UART.hdmarx, DMA_IT_HT);
	
	/*双板通讯*/
	HAL_UARTEx_ReceiveToIdle_DMA(&SERVENT_485_UART, uartServentRecBuffer, sizeof(uartServentRecBuffer));
	__HAL_DMA_DISABLE_IT(SERVENT_485_UART.hdmarx, DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&MASTER_485_UART, uartMasterRecBuffer, sizeof(uartMasterRecBuffer));
	__HAL_DMA_DISABLE_IT(MASTER_485_UART.hdmarx, DMA_IT_HT);	
}   
/**
 * @brief 串口dma不定长接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
			/*激光串口*/
	    if (huart == &LASER_UART){
				memcpy(distance_buffer, uart10RecBuffer, Size);
        distance_datacheck(distance_buffer);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer, sizeof(uart10RecBuffer));
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
		memset(uart5RecBuffer, 0, sizeof(uart5RecBuffer));
		HAL_UARTEx_ReceiveToIdle_DMA(&RC_UART, uart5RecBuffer, sizeof(uart5RecBuffer));
		__HAL_DMA_DISABLE_IT(RC_UART.hdmarx, DMA_IT_HT);		
	}
	/*算法串口*/
//	if(huart == &MINIPC_UART)
//	{
//		uint8_t temp_buffer[Size];
//		memcpy(temp_buffer, uart1RecBuffer, Size);
//		memset(uart1RecBuffer,0, MAX_RECEIVE_BUFFER_LENGTH);
//		UpperCommRecHandler(temp_buffer, Size);
//		static uint8_t error_count;
//		while(HAL_UARTEx_ReceiveToIdle_DMA(&MINIPC_UART, uart1RecBuffer, MAX_RECEIVE_BUFFER_LENGTH) == HAL_BUSY)
//		{

//			error_count ++;	
//			__HAL_UNLOCK(&MINIPC_UART);
//			if(error_count >= 4)
//				break;
//		}
//		error_count = 0;
//	}
	/*裁判串口*/
	if(huart == &REFEREE_UART)
	{
		uint8_t temp_buffer[Size];
		memcpy(temp_buffer, uart6RecBuffer, Size);
		memset(uart6RecBuffer, 0, sizeof(uart6RecBuffer));
		RefereeReceive(Size, temp_buffer);
		
		HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, sizeof(uart6RecBuffer));
		__HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);			
	}
	if(huart == &SERVENT_485_UART){
		/* 解析云台板下发的36字节帧: 0xA5 0x5A [float yaw 4B] [float pitch 4B] [float yaw_dps 4B] [float target_yaw 4B] [float target_pitch 4B] [6 * int16 fric_rpm] 0x0D 0x0A */
		static uint32_t rs485_rx_cnt = 0;	/* watch: 回调触发次数 */
		static uint32_t rs485_ok_cnt = 0;	/* watch: 帧解析成功次数 */
		static uint16_t rs485_last_size = 0; /* watch: 最近一次接收字节数 */
		rs485_rx_cnt++;
		rs485_last_size = Size;

		gimbal_yaw_rx_valid = 0;
		/* 协议帧固定36字节: 帧头0xA5 0x5A + 32B数据 + 帧尾0x0D 0x0A */
		if (Size >= 36U
			&& uartServentRecBuffer[0] == 0xA5
			&& uartServentRecBuffer[1] == 0x5A
			&& uartServentRecBuffer[34] == 0x0D
			&& uartServentRecBuffer[35] == 0x0A)
		{
			memcpy((void*)&gimbal_yaw_rx_d,   &uartServentRecBuffer[2], sizeof(float));
			memcpy((void*)&gimbal_pitch_rx_d, &uartServentRecBuffer[6], sizeof(float));
			memcpy((void*)&gimbal_yaw_dps_rx,  &uartServentRecBuffer[10], sizeof(float));
			memcpy((void*)&gimbal_yaw_target_rx_d, &uartServentRecBuffer[14], sizeof(float));
			memcpy((void*)&gimbal_pitch_target_rx_d, &uartServentRecBuffer[18], sizeof(float));
			for (uint8_t i = 0; i < 6; i++)
			{
				memcpy((void*)&gimbal_fric_rpm_rx_arr[i], &uartServentRecBuffer[22 + i * 2], sizeof(int16_t));
			}
			/* 兼容历史单摩擦轮变量 */
			gimbal_fric_rpm_rx = gimbal_fric_rpm_rx_arr[0];
			gimbal_yaw_rx_valid = 1;
			rs485_ok_cnt++;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&SERVENT_485_UART, uartServentRecBuffer, sizeof(uartServentRecBuffer));
		__HAL_DMA_DISABLE_IT(SERVENT_485_UART.hdmarx, DMA_IT_HT);
	}	
	// if(huart == &MASTER_485_UART){
		
	// 	HAL_UARTEx_ReceiveToIdle_DMA(&MASTER_485_UART, uartMasterRecBuffer, MAX_RECEIVE_BUFFER_LENGTH);	
	// }
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
	if (huart->Instance == USART10) {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer, sizeof(uart10RecBuffer));
		__HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
	}
//	else if (huart->Instance == MINIPC_UART.Instance)
//		HAL_UARTEx_ReceiveToIdle_DMA(&MINIPC_UART, uart1RecBuffer, MAX_RECEIVE_BUFFER_LENGTH);
	else if (huart->Instance == REFEREE_UART.Instance) {
		HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, sizeof(uart6RecBuffer));
		__HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);
	}
	else if (huart->Instance == SERVENT_485_UART.Instance) {
		HAL_UARTEx_ReceiveToIdle_DMA(&SERVENT_485_UART, uartServentRecBuffer, sizeof(uartServentRecBuffer));
		__HAL_DMA_DISABLE_IT(SERVENT_485_UART.hdmarx, DMA_IT_HT);
	}
	else if (huart->Instance == MASTER_485_UART.Instance) {
		HAL_UARTEx_ReceiveToIdle_DMA(&MASTER_485_UART, uartMasterRecBuffer, sizeof(uartMasterRecBuffer));
		__HAL_DMA_DISABLE_IT(MASTER_485_UART.hdmarx, DMA_IT_HT);
	}
	//一点点小史因为只有uart10出问题，不对，现在裁判也有问题了，可能是GND没接好
	/**/
	short temp=1;
	BaseType_t xHigherPriorityTaskWoken=0;
	//xQueueSend(g_musicQueue,&temp,portMAX_DELAY);这个有阻塞，不好
	xQueueSendFromISR(g_musicQueue, &temp, &xHigherPriorityTaskWoken);
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

	uint8_t temp=2;
	BaseType_t xHigherPriorityTaskWoken=0;
	xQueueSendFromISR(g_musicQueue, &temp, &xHigherPriorityTaskWoken);
}
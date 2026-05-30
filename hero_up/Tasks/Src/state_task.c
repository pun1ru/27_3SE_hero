
#include "tim.h"
#include "general_config_label.h"
#include "distance_check.h"
#include "general_task_include.h"
#include "iwdg.h"
/*---------------------------------------------------------------------------task monitor-----------------------------------------------------------------------------------*/
TaskMonitor taskMonitor;
TaskMonitor* _taskMonitor = &taskMonitor;

uint8_t taskIsBlockedOrDisturbedFlag = 0;

static void BlockOrDisturbDetect(uint16_t last_task_counter[]);

 /**
 * @brief 完成对其他任务的监测，若检测到有任务卡死，LED灯将不正常呼吸,相当于软件看门狗，监测是否有任务异常，
 * 		  在软件看门狗任务中喂硬件看门狗，监测软件看门狗是否正常
 
				顺便看一下电路连接是否正常
 *该功能已经削弱
 */
void MonitorTask(void* argument)
{
	static uint16_t last_total_task_counter[CREATE_TASK_NUM] = {0};
	TickType_t xLastWakeUpTime;	
	xLastWakeUpTime = xTaskGetTickCount();
	while(1)
	{
		/*喂硬件的狗*/
		HAL_IWDG_Refresh(&hiwdg1);
		/*软件看门狗*/
		BlockOrDisturbDetect(last_total_task_counter);
		/*连接看门狗*/
		
		
		/*若任务各标志位均为0，灯继续闪烁变化*/
		
		vTaskDelayUntil(&xLastWakeUpTime, MONITOR_TASK_PERIOD_SET);
	}
}
 
/**
 * @brief 记录上一个周期，各任务的counter
 */
static void BlockOrDisturbDetect(uint16_t last_task_counter[])
{
	uint16_t current_task_counter[CREATE_TASK_NUM] = {0};
	
	current_task_counter[REMOTE_RECEIVE_TASK_NUM] = *(_taskMonitor->TaskFrameCounterPtr._remote_rec_task);
	current_task_counter[STATE_TASK_NUM]  		  = *(_taskMonitor->TaskFrameCounterPtr._state_task);
	current_task_counter[DECISION_TASK_NUM] 	  = *(_taskMonitor->TaskFrameCounterPtr._decision_task);		
	current_task_counter[CONTROL_TASK_NUM]		  = *(_taskMonitor->TaskFrameCounterPtr._control_task);
	current_task_counter[IMU_TASK_NUM]			  = *(_taskMonitor->TaskFrameCounterPtr._imu_task);
	current_task_counter[DEBUG_TASK_NUM]		  = *(_taskMonitor->TaskFrameCounterPtr._debug_task);
	current_task_counter[UPPER_COMM_TASK_NUM]	  = *(_taskMonitor->TaskFrameCounterPtr._upper_pc_comm_task);
	current_task_counter[UI_OPERATION_TASK_NUM]	  = *(_taskMonitor->TaskFrameCounterPtr._ui_operation_task);
	current_task_counter[MUSIC_TASK_NUM]	  = *(_taskMonitor->TaskFrameCounterPtr._music_task);
	
	#if defined REMOTE_RECEIVE_TASK_NUM
		/*遥信号接收卡死检测，若超过三个软件看门狗周期无任何来源遥操作信号，遥操作事件组EVENT_GROUP_BIT_ERROR置位*/
		static uint8_t remote_noreceive_warning_count = 0;
		if(current_task_counter[REMOTE_RECEIVE_TASK_NUM] == last_task_counter[REMOTE_RECEIVE_TASK_NUM])
			remote_noreceive_warning_count++;
		else 
			remote_noreceive_warning_count = 0;
		if(remote_noreceive_warning_count > 3)
		{
			remote_noreceive_warning_count = 0;
			xEventGroupSetBits(remoteRecEventGroup, EVENT_GROUP_BIT_ERROR);//哦哦在这里
		}
		/*若超过十个软件看门狗周期遥控接收任务仍未刷新执行，则该任务卡死*/
		static uint8_t remote_block_warning_count = 0;
		if(current_task_counter[REMOTE_RECEIVE_TASK_NUM] == last_task_counter[REMOTE_RECEIVE_TASK_NUM])
			remote_block_warning_count++;
		else
		{
			remote_block_warning_count = 0;
			taskIsBlockedOrDisturbedFlag &= (~REMOTE_REC_TASK_MASK);//该任务标志位赋值为0
		}
		if(remote_block_warning_count > 10)
		{
			remote_block_warning_count = 0;
			taskIsBlockedOrDisturbedFlag |= REMOTE_REC_TASK_MASK;//该任务标志位赋值为1
		}
	#else
		taskIsBlockedOrDisturbedFlag &= (~REMOTE_REC_TASK_MASK);//该任务标志位赋值为0
	#endif
	
	/*以下为定周期任务检测是否卡死或周期扰乱*/
	/*若两次软件看门狗任务检测到定周期任务的counter无变化，说明卡死，因为软件看门狗任务周期为定周期任务周期的数倍;若检测到实际运行周期比设定周期大2ms以上，认为任务周期被扰乱*/
	#if defined STATE_TASK_NUM	
		if(current_task_counter[STATE_TASK_NUM] == last_task_counter[STATE_TASK_NUM] || ((*(_taskMonitor->TaskRunPeriodPtr._state_task))-STATE_TASK_PERIOD_SET) > 2 )
			taskIsBlockedOrDisturbedFlag |= STATE_TASK_MASK;	//赋值为1
		else
			taskIsBlockedOrDisturbedFlag &= (~STATE_TASK_MASK);	//赋值为0
	#else
		taskIsBlockedOrDisturbedFlag &= (~STATE_TASK_PERIOD_MASK);	//赋值为0
	#endif
	
	#if defined CONTROL_TASK_NUM
		if(current_task_counter[CONTROL_TASK_NUM] == last_task_counter[CONTROL_TASK_NUM] || ((*(_taskMonitor->TaskRunPeriodPtr._control_task))-CONTROL_TASK_PERIOD_SET) > 2 )
			taskIsBlockedOrDisturbedFlag |= CONTROL_TASK_MASK;
		else
			taskIsBlockedOrDisturbedFlag &= (~CONTROL_TASK_MASK);
	#else
		taskIsBlockedOrDisturbedFlag &= (~CONTROL_TASK_MASK);		
	#endif
	
	#if defined DECISION_TASK_NUM                                                                                                                                            	
		if(current_task_counter[DECISION_TASK_NUM] == last_task_counter[DECISION_TASK_NUM] || ((*(_taskMonitor->TaskRunPeriodPtr._decision_task))-DECISION_TASK_PERIOD_SET) > 2 )
			taskIsBlockedOrDisturbedFlag |= DECISION_TASK_MASK;
		else
			taskIsBlockedOrDisturbedFlag &= (~DECISION_TASK_MASK);
	#else
		taskIsBlockedOrDisturbedFlag &= (~DECISION_TASK_MASK);
	#endif
		
	#if defined IMU_TASK_NUM
		if(current_task_counter[IMU_TASK_NUM] == last_task_counter[IMU_TASK_NUM] || ((*(_taskMonitor->TaskRunPeriodPtr._imu_task))-IMU_TASK_PERIOD_SET) > 2 )
			taskIsBlockedOrDisturbedFlag |= IMU_TASK_MASK;
		else
			taskIsBlockedOrDisturbedFlag &= (~IMU_TASK_MASK);
	#else
		taskIsBlockedOrDisturbedFlag &= (~IMU_TASK_MASK);		
	#endif
		
	#if defined DEBUG_TASK_NUM	
		if(current_task_counter[DEBUG_TASK_NUM] == last_task_counter[DEBUG_TASK_NUM] || ((*(_taskMonitor->TaskRunPeriodPtr._debug_task))-DEBUG_TASK_PERIOD_SET) > 2 )
			taskIsBlockedOrDisturbedFlag |= DEBUG_TASK_MASK;
		else
			taskIsBlockedOrDisturbedFlag &= (~DEBUG_TASK_MASK);
	#else
		taskIsBlockedOrDisturbedFlag &= (~DEBUG_TASK_MASK);
	#endif
		
	#if defined UPPER_COMM_TASK_NUM	
		if(current_task_counter[UPPER_COMM_TASK_NUM] == last_task_counter[UPPER_COMM_TASK_NUM])
			taskIsBlockedOrDisturbedFlag |= UPPER_COMM_TASK_MASK;
		else
			taskIsBlockedOrDisturbedFlag &= (~UPPER_COMM_TASK_MASK);
	#else
		taskIsBlockedOrDisturbedFlag &= (~UPPER_COMM_TASK_MASK);
	#endif

	#if defined UI_OPERATION_TASK_NUM	
		if(current_task_counter[UI_OPERATION_TASK_NUM] == last_task_counter[UI_OPERATION_TASK_NUM] || ((*(_taskMonitor->TaskRunPeriodPtr._ui_operation_task))- UI_OPERATION_TASK_PERIOD_SET) > 2 )
			taskIsBlockedOrDisturbedFlag |= UI_OPERATION_TASK_MASK;
		else
			taskIsBlockedOrDisturbedFlag &= (~UI_OPERATION_TASK_MASK);
	#else
		taskIsBlockedOrDisturbedFlag &= (~UI_OPERATION_TASK_MASK);
	#endif	
	
	#if defined MUSIC_TASK_NUM	
		if(current_task_counter[MUSIC_TASK_NUM] == last_task_counter[MUSIC_TASK_NUM	] || ((*(_taskMonitor->TaskRunPeriodPtr._music_task))- MUSIC_TASK_PERIOD_SET) > 2 )
			taskIsBlockedOrDisturbedFlag |= MUSIC_TASK_MASK;
		else
			taskIsBlockedOrDisturbedFlag &= (~MUSIC_TASK_MASK);
	#else
		taskIsBlockedOrDisturbedFlag &= (~UI_OPERATION_TASK_MASK);
	#endif	
		
	memcpy(last_task_counter, current_task_counter, CREATE_TASK_NUM * sizeof(uint16_t));
}
/*---------------------------------------------------------------------------task state update-----------------------------------------------------------------------------------*/
static void StateUpdate(RobotState* robot_state, const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last);
static void StateUpdateFromNewRec(RobotState* robot_state, const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last);

RobotState robotState;
RobotState lastRobotState;
const RobotState* _robotState = &robotState;
const RobotState* _lastRobotState = &lastRobotState;
/*记录上一次遥控信号，用于判断边缘*/
static NormRemoteCmd normRemoteCmdLast;

/**
 * @brief 状态机任务，整车运动功能状态能且只能在这里进行修改操作
 */
void StateMachineTask(void* argument)
{	
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count = 0;
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._state_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._state_task = &this_tick_count;
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	while(1)
	{		
		/*任务主进程*/
		StateUpdate(&robotState, _normRemoteCmd, &normRemoteCmdLast);
		
		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;

		vTaskDelayUntil(&current_tick_count, STATE_TASK_PERIOD_SET);
	}
}
 
/**
 * @brief 全局状态初始化，可用作控制中断时状态的刷新
 */
static void StateInit(RobotState* robot_state)
{
	robot_state->ctrl_terminal = CONTROL_STOP;
	robot_state->chassis_mode = CHASSIS_FOLLOW;
	robot_state->fric_mode = FRIC_OFF;
	robot_state->stir_mode = STIR_LOCK;
	robot_state->auto_slope = 0;
	robot_state->follow=FOLLOW_OFF;//FOLLOW应该是SNIPER的子状态
	robot_state->sniper=SNIPER_OFF;
	robot_state->lens=LENS_OFF;//这个看情况
	robot_state->cam_target=CAM_TARGET_MID;
	robot_state->mouse_fix=MOUSE_FIX_OFF;
	robot_state->aim_mode=0;
	robot_state->joint_mode = ROBOT_JOINT_MODE_NORMAL;
	robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
	robot_state->jump_mode = ROBOT_JUMP_MODE_OFF;
	worldGimbal.enable = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}

/**
 * @brief 全局状态更新（根据标准控制指令）
 */
void xvni_42_heart_leng();
static void StateUpdate(RobotState* robot_state, const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last)
{
	xvni_42_heart_leng();
	
	/* 下板血量保护：485双板通信收到HP归零时进入保护态 */
	if(servant485_hp_zero_flag){
		StateInit(robot_state);
		StateInit(&lastRobotState);
		servant485_hp_zero_flag = 0;
		return;
	}
	
	switch(norm_remote_cmd->remote_source)
	{
		case ERROR_RECEIVE:
			StateInit(robot_state);
			StateInit(&lastRobotState);
		break;
	
		case DT7:
			StateUpdateFromNewRec(robot_state, norm_remote_cmd, &normRemoteCmdLast);
		break;
		
		case VT13:
			
		break;
		default:
			StateInit(robot_state);
			StateInit(&lastRobotState);
	}
	memcpy(&normRemoteCmdLast, norm_remote_cmd, sizeof(NormRemoteCmd));
}

/**
 * @brief DT7遥控器指令状态解读
 */
/*一些答辩全局变量，早晚给他包装成结构体*/
//uint8_t lastRobotState.follow;
//uint8_t follow_leap_flag; 
//uint8_t lastRobotState.lens=SNIPER_OFF;
//uint8_t aacheck=0;
//uint8_t lastRobotState.sniper=0;

int xvni=0;
void xvni_42_heart_leng(){
	float temp_cold=((float)ext_game_robot_status.shooter_barrel_cooling_value/100.0);//读取每秒冷却值，变成每十毫秒冷却
	xvni=xvni-temp_cold;
	if(xvni<0)//如果到0了
		 xvni=0;
}
//打一次的虚拟热量结算
void xvni_42_heart_da(){
	if(xvni<ext_game_robot_status.shooter_barrel_heat_limit)
		xvni+=100;
//	if(xvni>=ext_game_robot_status.shooter_barrel_heat_limit)
//		xvni=ext_game_robot_status.shooter_barrel_heat_limit;
}

extern GimbalControl gimbalControl;
extern ChassisControl chassisControl;
extern UpperComputerComm upperComputerComm;
extern WorldGimbal worldGimbal;
int crawler_rotate_flag = 0;
uint8_t yaw_recoil_compensation_signal=0;
float wait_see=0;
uint8_t state_machine_debug_mode = 0;
uint8_t shit_last_PC_Receive_shoot_mode=0;//算法上升沿记录函数 
static void StateUpdateFromNewRec(RobotState* robot_state, const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last)
{
	static uint8_t separate_count = 0;
	static uint8_t preclimb_auto_climb = 0u;
	lastRobotState.follow=robot_state->follow;

	/* 双上拨时用ch4切换调试状态机，同时保持保护态 */
	if(norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP && norm_remote_cmd->Switch.switch_R1 == NORM_RC_SW_UP)
	{
		if(norm_remote_cmd->RelativeCH.ch4 > 0.1f)
			state_machine_debug_mode = 0;
		else if(norm_remote_cmd->RelativeCH.ch4 < -0.1f)
			state_machine_debug_mode = 1;

		StateInit(robot_state);
		StateInit(&lastRobotState);
		return;
	}

	if(state_machine_debug_mode == 1)
	{
		/* 新状态机：保留比赛入口，R1中/下仅切换joint_mode */
		robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
		robot_state->jump_mode = ROBOT_JUMP_MODE_OFF;
		worldGimbal.enable = 0;
		switch(norm_remote_cmd->Switch.switch_R1)
		{
			case NORM_RC_SW_UP:
				// preclimb_auto_climb = 0u;
				// /* 双拨上保护（冗余保护） */
				// if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_UP)
				// {
				// 	StateInit(robot_state);
				// 	StateInit(&lastRobotState);
				// }
				// /* 右上左中保留PC比赛逻辑入口，ch4>0.1切换到陀螺旋转 */
				// if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_MID)
				// {
				// 	robot_state->ctrl_terminal = CONTROL_FROM_PC;
				// 	if (norm_remote_cmd->RelativeCH.ch4 > 0.1f)
				// 		robot_state->chassis_mode = CHASSIS_REVOLVE;
				// 	else if (norm_remote_cmd->RelativeCH.ch4 < -0.1f)
				// 		robot_state->chassis_mode = CHASSIS_FOLLOW;
				// }
			break;

			case NORM_RC_SW_MID:
				// preclimb_auto_climb = 0u;
				// robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;
				// robot_state->joint_mode = ROBOT_JOINT_MODE_NORMAL;
				// if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_MID)
				// 	robot_state->stand_mode = JointStairUpIsDetected() ? ROBOT_STAND_MODE_STAIR_UP : ROBOT_STAND_MODE_PRE_STAIR;
				// else if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_DOWN)
				// 	robot_state->stand_mode = ROBOT_STAND_MODE_STAIR_UP;

				// if ((robot_state->stand_mode == ROBOT_STAND_MODE_PRE_STAIR ||
				//      robot_state->stand_mode == ROBOT_STAND_MODE_STAIR_UP) &&
				//     norm_remote_cmd->RelativeCH.ch1 > 0.1f)
				// 	crawler_rotate_flag = 1;
				// else if ((robot_state->stand_mode == ROBOT_STAND_MODE_PRE_STAIR ||
				//           robot_state->stand_mode == ROBOT_STAND_MODE_STAIR_UP) &&
				//          norm_remote_cmd->RelativeCH.ch1 < -0.1f)
				// 	crawler_rotate_flag = -1;
				// else
				// 	crawler_rotate_flag = 0;
			  break;
         //右下左中进入爬坡模式
		 //右中左中升高
		 //右中左下上台阶,右下左下进入跳跃模式
			case NORM_RC_SW_DOWN:
				// robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
				// robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;
				// if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP)
				// {
				// 	const float preclimb_switch_angle_rad = -2.50f;
				// 	const float preclimb_restore_pitch_d = 16.0f;
				// 	float pitch_abs_d = g_joint_body_pitch_ctrl_d;

				// 	if (preclimb_auto_climb)
				// 	{
				// 		robot_state->joint_mode = ROBOT_JOINT_MODE_CLIMB;
				// 		if (pitch_abs_d < preclimb_restore_pitch_d)
				// 		{
				// 			preclimb_auto_climb = 0u;
				// 			robot_state->joint_mode = ROBOT_JOINT_MODE_PRECLIMB;
				// 		}
				// 	}
				// 	else
				// 	{
				// 		robot_state->joint_mode = ROBOT_JOINT_MODE_PRECLIMB;
				// 		if (jointControl.JointEstimate.motor_angles_rad[LEG_LF] < preclimb_switch_angle_rad &&
				// 		    jointControl.JointEstimate.motor_angles_rad[LEG_RF] < preclimb_switch_angle_rad &&
				// 		    pitch_abs_d>16.0
				// 		)
				// 		{
				// 			preclimb_auto_climb = 1u;
				// 			robot_state->joint_mode = ROBOT_JOINT_MODE_CLIMB;
				// 		}
				// 	}
				// }
				// else if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_DOWN)
				// {
				// 	preclimb_auto_climb = 0u;
				// 	robot_state->joint_mode = ROBOT_JOINT_MODE_OUTCLIMB;
				// }
				// else if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_MID)
				// {
				// 	preclimb_auto_climb = 0u;
				// 	robot_state->joint_mode = ROBOT_JOINT_MODE_CLIMB;
				// }
			if (_normRemoteCmd->RelativeCH.ch1> 0.1f&&(robot_state->joint_mode == ROBOT_JOINT_MODE_CLIMB||robot_state->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB))
			{
//			if(norm_remote_cmd->RelativeCH.ch4>0.1)
				crawler_rotate_flag = 1;
			}
			else
				crawler_rotate_flag = 0;
			break;

			default:
			break;
		}
	}
	else
	{
	robot_state->jump_mode = ROBOT_JUMP_MODE_OFF;
	switch(norm_remote_cmd->Switch.switch_R1)
	  {
		case NORM_RC_SW_UP:
			/*双拨上保护*/
			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_UP){
				StateInit(robot_state);	
				StateInit(&lastRobotState);
			}
			
			/*右上左中进入PC比赛模式*/
			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_MID){
				robot_state->ctrl_terminal = CONTROL_FROM_PC;
//			if(ext_game_robot_status.current_HP <= 0)
//				StateInit(robot_state);

			// // V: 系统复位
			// 	if(norm_remote_cmd->PCKeyBoard.level_key_V && !norm_remote_cmd_last->PCKeyBoard.level_key_V){
			// 		HAL_NVIC_SystemReset();
			// 	}

// X: 开关吊射模式 (sniper)，PC模式下自动同步世界系开关
			if(norm_remote_cmd->PCKeyBoard.level_key_X && !norm_remote_cmd_last->PCKeyBoard.level_key_X)
			{
				gimbalControl.GimbalTargetInput.yaw_recoil_compensation_d = 0;
				robot_state->sniper ^= 1;
				/* PC sniper_on 默认走世界系控制 */
				if (robot_state->sniper == SNIPER_ON) {
					WorldGimbalAlignToCurrent(&worldGimbal);
					worldGimbal.enable = 1;
				} else {
					worldGimbal.enable = 0;
				}
				}

            // C: 吊射模式下，按一次追逐一次上位机目标角度（单次触发）
				if(norm_remote_cmd->PCKeyBoard.level_key_C && !norm_remote_cmd_last->PCKeyBoard.level_key_C && robot_state->sniper == SNIPER_ON){
					robot_state->aim_mode = 1;
				}

			// Z: 切换摄影目标端 (上→中→下→上)纵向画幅
				if(norm_remote_cmd->PCKeyBoard.level_key_Z && !norm_remote_cmd_last->PCKeyBoard.level_key_Z){
					robot_state->cam_target++;
					if(robot_state->cam_target > CAM_TARGET_DOWN)
						robot_state->cam_target = CAM_TARGET_UP;
				}

			// E: 切换mouse_fix模式（sniper_on下生效，锁止鼠标输入仅保留WASD）
				if(norm_remote_cmd->PCKeyBoard.level_key_E && !norm_remote_cmd_last->PCKeyBoard.level_key_E){
					robot_state->mouse_fix ^= 1;
				}

			// F: 摩擦轮开关 (fric_mode)
				if(norm_remote_cmd->PCKeyBoard.level_key_F && !norm_remote_cmd_last->PCKeyBoard.level_key_F)
					robot_state->fric_mode++;

			// Q: sniper_on→世界系pitch 39度；否则切换底盘模式
				if(norm_remote_cmd->PCKeyBoard.level_key_Q && !norm_remote_cmd_last->PCKeyBoard.level_key_Q)
				{
					if(robot_state->sniper == SNIPER_ON)
					{
						if (worldGimbal.enable) {
							WorldGimbalSetWorldAngles(&worldGimbal,
								worldGimbal.WorldGimbalEstimate.world_yaw_deg, 40.0f);
						} else {
							gimbalControl.GimbalTargetInput.small_pitch_angle_d = 40.0f;
							gimbalControl.GimbalTargetInput.pitch_angle_d = 40.0f;
						}
					}
					else
					{
						if(robot_state->chassis_mode == CHASSIS_FOLLOW)
							robot_state->chassis_mode = CHASSIS_REVOLVE;
						else if(robot_state->chassis_mode == CHASSIS_REVOLVE)
							robot_state->chassis_mode = CHASSIS_FOLLOW;
					}
				}

			// G: 一键掉头 (FOLLOW_BACK)，再按一次换回来
				if(norm_remote_cmd->PCKeyBoard.level_key_G && !norm_remote_cmd_last->PCKeyBoard.level_key_G)
				{
					if(robot_state->chassis_mode == CHASSIS_FOLLOW_BACK)
						robot_state->chassis_mode = CHASSIS_FOLLOW;
					else
						robot_state->chassis_mode = CHASSIS_FOLLOW_BACK;
				}
			// R: sniper_on→一键关机电脑标志位 (reserved[0] bit0置1，发送后由UpperPCCommTask清零)
				if(norm_remote_cmd->PCKeyBoard.level_key_R && !norm_remote_cmd_last->PCKeyBoard.level_key_R)
				{
					if(robot_state->sniper == SNIPER_ON)
						upperComputerComm.Send.reserved[0] |= 0x01;
				}
			// V: 预备上台阶 (PRE_STAIR -> STAIR_UP)，再按一次回到普通模式
				if(norm_remote_cmd->PCKeyBoard.level_key_V && !norm_remote_cmd_last->PCKeyBoard.level_key_V)
				{
					if(robot_state->stand_mode == ROBOT_STAND_MODE_NORMAL)
						robot_state->stand_mode = ROBOT_STAND_MODE_PRE_STAIR;
					else if(robot_state->stand_mode == ROBOT_STAND_MODE_PRE_STAIR || robot_state->stand_mode == ROBOT_STAND_MODE_STAIR_UP)
						robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
				}

			// B: sniper_on→切换横向画幅(左1→左2→右1→右2 via reserved[1]: 0x01-0x04); 普通模式→预备下台阶
				if(norm_remote_cmd->PCKeyBoard.level_key_B&& !norm_remote_cmd_last->PCKeyBoard.level_key_B)
				{
					if (robot_state->sniper == SNIPER_ON) {
						/* 横向画幅循环: 0x01→0x02→0x03→0x04→0x01 */
						uint8_t *p = &upperComputerComm.Send.reserved[1];
						*p = (*p >= 0x04) ? 0x01 : (*p + 1);
					} else {
						if(robot_state->stand_mode == ROBOT_STAND_MODE_NORMAL)
							robot_state->stand_mode = ROBOT_STAND_MODE_PRE_DOWN_STAIR;
						else if(robot_state->stand_mode == ROBOT_STAND_MODE_PRE_DOWN_STAIR)
							robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
					}
				}

				if(robot_state->sniper == SNIPER_OFF)
					distance_check.distance_check_translate.angle = 0;
			}
			separate_count = 0;
			break;
		case NORM_RC_SW_MID:
			robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;

		//摩擦轮未开状态向下拨两次进入分离状态 没有删除
			if(robot_state->fric_mode == FRIC_OFF && normRemoteCmdLast.Switch.switch_L1 != NORM_RC_SW_DOWN && norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_DOWN)
				separate_count++;
			if(separate_count > 2)
				separate_count = 0;
			if((norm_remote_cmd->RelativeCH.ch4) > 0.1){
				// robot_state->chassis_mode = CHASSIS_REVOLVE;
				}
			else if(separate_count == 2)
				robot_state->chassis_mode = CHASSIS_FOLLOW;
			else
				robot_state->chassis_mode = CHASSIS_FOLLOW;
		
			/*拨轮控制跟随方向：>0.1正向跟随，<-0.1反向180度跟随 */
			if(norm_remote_cmd->RelativeCH.ch4 > 0.1){
				robot_state->chassis_mode = CHASSIS_FOLLOW;
			}
			else if(norm_remote_cmd->RelativeCH.ch4 < -0.1){
				robot_state->chassis_mode = CHASSIS_FOLLOW_BACK;
			}
			else {
				robot_state->auto_slope = 0;
			}
			//遥控器左拨下 脉冲触发
			if(normRemoteCmdLast.Switch.switch_L1 != NORM_RC_SW_UP && norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP)
				robot_state->fric_mode++;
			
			if(_shootControl->ShootEstimate.stir_block_flag)
					robot_state->stir_mode = STIR_REVERSE;
				
			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_DOWN && robot_state->fric_mode != FRIC_OFF)
			{
				robot_state->stir_mode = STIR_ANGLE_CONTROL;
			}			
			else
				robot_state->stir_mode = STIR_LOCK;
			
			//robot_state->fric_mode=1;这个除了我不要取消注释
			//他妈的记得注释
			/*右拨中退出吊射模式*/
			robot_state->sniper=SNIPER_OFF;
			robot_state->follow=FOLLOW_OFF ;
			worldGimbal.enable = 0;
		break;
		
		case NORM_RC_SW_DOWN:
			//给算法调试的时候不要右播下吊射模式
			/*右下进入遥控器吊射模式*/
			robot_state->sniper=SNIPER_ON;

			robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;
		//死亡时进入保护状态
		/*调试用，在遥控器右下也可以用遥控器打弹*/
		if(normRemoteCmdLast.Switch.switch_L1 != NORM_RC_SW_UP && norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP)
				robot_state->fric_mode++;
			
			if(_shootControl->ShootEstimate.stir_block_flag)
					robot_state->stir_mode = STIR_REVERSE;
			
		//调试用，待会取消回来
		int mardio;
			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_DOWN && robot_state->fric_mode != FRIC_OFF)
			{
				robot_state->stir_mode = STIR_ANGLE_CONTROL;
			}			
			else
				robot_state->stir_mode = STIR_LOCK;
		
			/*右拨下开镜*/
				robot_state->sniper=SNIPER_ON;
			/*遥控器拨轮边沿切换世界系模式（CH4>0.1或<-0.1拨一次切换，不需保持）*/
			if ((normRemoteCmdLast.RelativeCH.ch4 <= 0.1f && norm_remote_cmd->RelativeCH.ch4 > 0.1f) ||
			    (normRemoteCmdLast.RelativeCH.ch4 >= -0.1f && norm_remote_cmd->RelativeCH.ch4 < -0.1f))
			{
				if (worldGimbal.enable) {
					worldGimbal.enable = 0;
				} else {
					WorldGimbalAlignToCurrent(&worldGimbal);
					worldGimbal.enable = 1;
				}
			}
	}

 }
}

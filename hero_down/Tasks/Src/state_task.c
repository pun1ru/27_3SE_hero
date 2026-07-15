#include "tim.h"
#include "general_config_label.h"
#include "general_task_include.h"
#include "iwdg.h"
/*---------------------------------------------------------------------------task monitor-----------------------------------------------------------------------------------*/
TaskMonitor taskMonitor;
TaskMonitor* _taskMonitor = &taskMonitor;

/* 控制链延迟监控全局实例 */
CtrlChainTimer g_chain_timer = {0};

extern JointControl jointControl;
extern float g_joint_body_pitch_ctrl_d;

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
	current_task_counter[ESTIMATE_TASK_NUM]	  = *(_taskMonitor->TaskFrameCounterPtr._estimate_task);
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
		
	#if defined ESTIMATE_TASK_NUM
    if(current_task_counter[ESTIMATE_TASK_NUM] == last_task_counter[ESTIMATE_TASK_NUM]
        || ((*(_taskMonitor->TaskRunPeriodPtr._estimate_task)) - ESTIMATE_TASK_PERIOD_SET) > 2)
        taskIsBlockedOrDisturbedFlag |= ESTIMATE_TASK_MASK;
    else
        taskIsBlockedOrDisturbedFlag &= (~ESTIMATE_TASK_MASK);
	#else
    taskIsBlockedOrDisturbedFlag &= (~ESTIMATE_TASK_MASK);
#endif
	memcpy(last_task_counter, current_task_counter, CREATE_TASK_NUM * sizeof(uint16_t));
}
/*---------------------------------------------------------------------------task state update-----------------------------------------------------------------------------------*/
static void StateUpdate(const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last);

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
		StateUpdate(_normRemoteCmd, &normRemoteCmdLast);
		
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
	robot_state->joint_mode = ROBOT_JOINT_MODE_NORMAL;
	robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
	robot_state->jump_mode = ROBOT_JUMP_MODE_OFF;
	robot_state->mouse_fix = MOUSE_FIX_OFF;
	robot_state->cam_target = CAM_TARGET_MID;
	robot_state->world_enable = WORLD_ENABLE_OFF;
}

/**
 * @brief 全局状态更新（根据标准控制指令）
 */
void xvni_42_heart_leng();
// static void StateUpdate(RobotState* robot_state, const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last)
// {
// 	xvni_42_heart_leng();
// 	switch(norm_remote_cmd->remote_source)
// 	{
// 		case ERROR_RECEIVE:
// 			StateInit(robot_state);
// 			StateInit(&lastRobotState);
// 		break;
	
// 		case DT7:
// 			StateUpdateFromNewRec(robot_state, norm_remote_cmd, &normRemoteCmdLast);
// 		break;
		
// 		case VT13:
			
// 		break;
// 		default:
// 			StateInit(robot_state);
// 			StateInit(&lastRobotState);
// 	}
// 	memcpy(&normRemoteCmdLast, norm_remote_cmd, sizeof(NormRemoteCmd));
// }

/**
 * @brief DT7遥控器指令状态解读
 */
/*一些答辩全局变量，早晚给他包装成结构体*/

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
extern uint16_t stall_count;  /* 拨盘堵转计数, 用于测试触发堵转恢复 */
extern GimbalControl gimbalControl;
extern ChassisControl chassisControl;
int crawler_rotate_flag = 0;
uint8_t yaw_recoil_compensation_signal=0;
float wait_see=0;
uint8_t state_machine_debug_mode = 0;
uint8_t shit_last_PC_Receive_shoot_mode=0;//算法上升沿记录函数

/* 静态事件 */
static QEvt evtSwitchProtected  = { .sig = SWITCH_FINAL_SIG };
static QEvt evtSwitchPC         = { .sig = Switch_PC_SIG };
static QEvt evtSwitchRC         = { .sig = Switch_RC_SIG };
static QEvt evtSwitchSniper     = { .sig = SWITCH_SNIPER_SIG };
static QEvt evtSwitchRevolve    = { .sig = SWITCH_REVOLVE_SIG };
static QEvt evtSwitchBack       = { .sig = SWITCH_BACK_SIG };
static QEvt evtSwitchStair      = { .sig = SWITCH_STAIR_SIG };
static QEvt evtRCLost           = { .sig = RC_LOST_SIG };
static void DecisionInputPost(const NormRemoteCmd* cmd, const NormRemoteCmd* cmd_last)
{
    DecisionAO* ao = &DecisionAO_inst;  /* 直接写AO实例（非const） */
    uint8_t L1     = cmd->Switch.switch_L1;
    uint8_t R1     = cmd->Switch.switch_R1;
    uint8_t L1_last = cmd_last->Switch.switch_L1;
    uint8_t R1_last = cmd_last->Switch.switch_R1;

    /* ===== 1. 双上拨保护（最高优先级）===== */
    if (L1 == NORM_RC_SW_UP && R1 == NORM_RC_SW_UP)
    {
        ao->ctrl_terminal = CTRL_STOP;
        ao->chassis_mode  = CHS_FOLLOW;
        ao->sniper        = SNIPER_OFF;
        ao->fric_mode     = FRIC_OFF;
        ao->stir_mode     = STIR_LOCK;
        ao->joint_mode    = JOINT_NORMAL;
        ao->stand_mode    = STAND_NORMAL;
        ao->mouse_fix     = MOUSE_FIX_OFF;
        ao->cam_target    = CAM_TARGET_MID;
        ao->world_enable  = WORLD_ENABLE_OFF;
        ao->can_enable    = CAN_DISABLE;
        QACTIVE_POST(AO_DecisionAO, &evtSwitchProtected, 0);
        return;
    }

    /* ===== 2. 开关组合变化 → 控制终端切换 ===== */
    if (R1 != R1_last || L1 != L1_last)
    {
        if (L1 == NORM_RC_SW_MID && R1 == NORM_RC_SW_UP)
        {
            QACTIVE_POST(AO_DecisionAO, &evtSwitchPC, 0);
        }
        else
        {
            QACTIVE_POST(AO_DecisionAO, &evtSwitchRC, 0);
        }
    }

    /* ===== 3. PC按键上升沿 → 有信号的发事件 / 没信号的直接写 ===== */

    /* --- 3a. 状态迁移事件 --- */
    if (cmd->PCKeyBoard.level_key_X && !cmd_last->PCKeyBoard.level_key_X)
    {
        QACTIVE_POST(AO_DecisionAO, &evtSwitchSniper, 0);
    }

    if (cmd->PCKeyBoard.level_key_Q && !cmd_last->PCKeyBoard.level_key_Q)
    {
        QACTIVE_POST(AO_DecisionAO, &evtSwitchRevolve, 0);
    }

    if (cmd->PCKeyBoard.level_key_G && !cmd_last->PCKeyBoard.level_key_G)
    {
        QACTIVE_POST(AO_DecisionAO, &evtSwitchBack, 0);
    }

    if (cmd->PCKeyBoard.level_key_V && !cmd_last->PCKeyBoard.level_key_V)
    {
        QACTIVE_POST(AO_DecisionAO, &evtSwitchStair, 0);
    }

    /* --- 3b. 直接写字段（无状态迁移，仅改AO属性）--- */

    /* F: 摩擦轮开关 toggle */
    if (cmd->PCKeyBoard.level_key_F && !cmd_last->PCKeyBoard.level_key_F)
    {
        ao->fric_mode = (ao->fric_mode == FRIC_ON) ? FRIC_OFF : FRIC_ON;
    }

    /* E: 鼠标锁定 toggle */
    if (cmd->PCKeyBoard.level_key_E && !cmd_last->PCKeyBoard.level_key_E)
    {
        ao->mouse_fix = !ao->mouse_fix;
    }

    /* Z: 摄影目标循环 MID→UP→DOWN→MID */
    if (cmd->PCKeyBoard.level_key_Z && !cmd_last->PCKeyBoard.level_key_Z)
    {
        if (ao->cam_target == CAM_TARGET_MID)
            ao->cam_target = CAM_TARGET_UP;
        else if (ao->cam_target == CAM_TARGET_UP)
            ao->cam_target = CAM_TARGET_DOWN;
        else
            ao->cam_target = CAM_TARGET_MID;
    }

    /* B: sniper_off→下坡模式 toggle（原ROBOT_STAND_MODE_PRE_DOWN_STAIR） */
    if (cmd->PCKeyBoard.level_key_B && !cmd_last->PCKeyBoard.level_key_B)
    {
        /* TODO: PRE_DOWN_STAIR 在新HSM中暂未建模，需要先在QM中添加状态 */
    }

    /* C: sniper_on→aim_mode追逐; sniper_off→joint_mode toggle（joint_mode暂不用） */
    if (cmd->PCKeyBoard.level_key_C && !cmd_last->PCKeyBoard.level_key_C)
    {
        if (ao->sniper == SNIPER_ON)
        {
            /* aim_mode = 1 留给gimbalControl消费，AO不管理aim_mode字段 */
        }
        /* else: joint_mode toggle 已废弃 */
    }

    /* ===== 4. 遥控器模式下的拨杆边缘 ===== */

    /* 左拨杆上升沿 → 摩擦轮 toggle（排除双上拨保护态） */
    if (L1 == NORM_RC_SW_UP && L1 != L1_last
        && !(L1 == NORM_RC_SW_UP && R1 == NORM_RC_SW_UP))
    {
        ao->fric_mode = (ao->fric_mode == FRIC_ON) ? FRIC_OFF : FRIC_ON;
    }

    /* ===== 5. 遥控器模式：左拨杆下 + 摩擦轮开 → 持续拨弹 ===== */
    if (!(L1 == NORM_RC_SW_MID && R1 == NORM_RC_SW_UP))  /* 排除PC模式 */
    {
        if (L1 == NORM_RC_SW_DOWN && ao->fric_mode != FRIC_OFF)
        {
            ao->stir_mode = STIR_ANGLE_CONTROL;
        }
        else if (!_shootControl->ShootEstimate.stir_block_flag)
        {
            ao->stir_mode = STIR_LOCK;
        }

        if (_shootControl->ShootEstimate.stir_block_flag)
        {
            ao->stir_mode = STIR_REVERSE;
        }
    }

    /* ===== 6. 右拨杆边缘：进入/离开右下 → toggle狙击 ===== */
    if (!(L1 == NORM_RC_SW_MID && R1 == NORM_RC_SW_UP))  /* 排除PC模式 */
    {
        if ((R1 == NORM_RC_SW_DOWN && R1_last != NORM_RC_SW_DOWN)
            || (R1 != NORM_RC_SW_DOWN && R1_last == NORM_RC_SW_DOWN))
        {
            QACTIVE_POST(AO_DecisionAO, &evtSwitchSniper, 0);
        }
    }

    /* ===== 7. stir_mode 运行时覆盖（HSM entry设默认值，这里按实时条件覆盖）===== */
    /* PC模式：鼠标左键 + 摩擦轮开 + 热量有余量 → 触发单发 */
    if (cmd->PCMouse.mouse_left
        && ao->fric_mode == FRIC_ON
        && (ext_game_robot_status.shooter_barrel_heat_limit
            - ext_power_heat_data.shooter_42mm_barrel_heat >= 100))
    {
        ao->stir_mode = STIR_ANGLE_CONTROL;
    }

    /* 拨盘堵转恢复 */
    if (_shootControl->ShootEstimate.stir_block_flag)
    {
        ao->stir_mode = STIR_REVERSE;
    }

    /* ===== 8. 右中退出狙击模式（遥控器从右下拨回右中 → 关狙击）===== */
    if (R1 == NORM_RC_SW_MID && R1_last == NORM_RC_SW_DOWN)
    {
        /* 遥控器退出右下 → HSM正常会处理（Switch_RC_SIG在组合变化时已发） */
        /* sniper的关闭由NormalMode entry的SNIPER_OFF保证 */
    }
}

/**
 * @brief 全局状态更新（替代旧StateUpdateFromNewRec）
 * @note  全部状态由 DecisionAO 管理，各模块通过 pDecisionAO 直接读取。
 *
 *        RC_OK / RC_LOST：
 *        - ERROR_RECEIVE → 发RC_LOST_SIG，AO进入Protected
 *        - DT7正常接收 → 发DecisionInputPost，AO正常运行
 *        - 不需要手动设rc_lost_flag，HSM内部管理
 */
static void StateUpdate(const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last)
{
    DecisionAO* ao = &DecisionAO_inst;

    switch (norm_remote_cmd->remote_source)
    {
        case ERROR_RECEIVE:
        default:
        {
            /* RC信号丢失 → 直接写flag + 通知AO回收Protected */
            ao->rc_lost_flag = RC_LOST;
            QACTIVE_POST(AO_DecisionAO, &evtRCLost, 0);
            break;
        }
        case DT7:
            ao->rc_lost_flag = RC_OK;
            DecisionInputPost(norm_remote_cmd, norm_remote_cmd_last);
            break;
        case VT13:
            break;
    }
    memcpy(norm_remote_cmd_last, norm_remote_cmd, sizeof(NormRemoteCmd));
}

// static void StateUpdateFromNewRec(RobotState* robot_state, const NormRemoteCmd* norm_remote_cmd, NormRemoteCmd* norm_remote_cmd_last)
// {
// 	static uint8_t separate_count = 0;
// 	static uint8_t preclimb_auto_climb = 0u;
// 	lastRobotState.follow=robot_state->follow;

// 	/* 双上拨时用ch4切换调试状态机，同时保持保护态 */
// 	if(norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP && norm_remote_cmd->Switch.switch_R1 == NORM_RC_SW_UP)
// 	{
// 		if(norm_remote_cmd->RelativeCH.ch4 > 0.1f)
// 			state_machine_debug_mode = 0;
// 		else if(norm_remote_cmd->RelativeCH.ch4 < -0.1f)
// 			state_machine_debug_mode = 1;

// 		StateInit(robot_state);
// 		StateInit(&lastRobotState);
// 		return;
// 	}

// 	if(state_machine_debug_mode == 1)
// 	{
// 		/* 新状态机：保留比赛入口，R1中/下仅切换joint_mode */
// 		robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
// 		robot_state->jump_mode = ROBOT_JUMP_MODE_OFF;
// 		switch(norm_remote_cmd->Switch.switch_R1)
// 		{
// 			case NORM_RC_SW_UP:
// 				preclimb_auto_climb = 0u;
// 				/* 双拨上保护（冗余保护） */
// 				if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_UP)
// 				{
// 					StateInit(robot_state);
// 					StateInit(&lastRobotState);
// 				}
// 				/* 右上左中保留PC比赛逻辑入口，ch4>0.1切换到陀螺旋转 */
// 				if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_MID)
// 				{
// 					robot_state->ctrl_terminal = CONTROL_FROM_PC;
// 					if (norm_remote_cmd->RelativeCH.ch4 > 0.1f)
// 						robot_state->chassis_mode = CHASSIS_REVOLVE;
// 					else if (norm_remote_cmd->RelativeCH.ch4 < -0.1f)
// 						robot_state->chassis_mode = CHASSIS_FOLLOW;
// 				}
// 			break;

// 			case NORM_RC_SW_MID:
// 				preclimb_auto_climb = 0u;
// 				robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;
// 				robot_state->joint_mode = ROBOT_JOINT_MODE_NORMAL;

// 				if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_MID)
// 					robot_state->stand_mode = JointStairUpIsDetected() ? ROBOT_STAND_MODE_STAIR_UP : ROBOT_STAND_MODE_PRE_STAIR;
// 				else if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_DOWN)
// 					robot_state->stand_mode = ROBOT_STAND_MODE_STAIR_UP;

// 				if ((robot_state->stand_mode == ROBOT_STAND_MODE_PRE_STAIR ||
// 				     robot_state->stand_mode == ROBOT_STAND_MODE_STAIR_UP) &&
// 				    norm_remote_cmd->RelativeCH.ch1 > 0.1f)
// 					crawler_rotate_flag = 1;
// 				else if ((robot_state->stand_mode == ROBOT_STAND_MODE_PRE_STAIR ||
// 				          robot_state->stand_mode == ROBOT_STAND_MODE_STAIR_UP) &&
// 				         norm_remote_cmd->RelativeCH.ch1 < -0.1f)
// 					crawler_rotate_flag = -1;
// 				else
// 					crawler_rotate_flag = 0;
// 			  break;
//          //右下左上进入上坡模式(CLIMB)
// 		case NORM_RC_SW_DOWN:
// 			preclimb_auto_climb = 0u;
// 			robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;
// 			robot_state->joint_mode = ROBOT_JOINT_MODE_NORMAL;
// 			/* 上坡模式：右下左上触发，走CLIMB功率分配+电容加压 */
// 			if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP)
// 				robot_state->joint_mode = ROBOT_JOINT_MODE_CLIMB;
// 			crawler_rotate_flag = 0;
// 		  break;

// 			default:
// 			break;
// 		}
// 	}
// 	else
// 	{
// 	robot_state->jump_mode = ROBOT_JUMP_MODE_OFF;
// 	switch(norm_remote_cmd->Switch.switch_R1)
// 	{
// 		case NORM_RC_SW_UP:
// 			/*双拨上保护*/
// 			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_UP){
// 				StateInit(robot_state);	
// 				StateInit(&lastRobotState);
// 			}
			
// 			/*右上左中进入PC比赛模式*/
// 			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_MID){
// 				robot_state->ctrl_terminal = CONTROL_FROM_PC;
// 			if(ext_game_robot_status.current_HP <= 0)
// 				StateInit(robot_state);
// 			// // V: 系统复位
// 			// 	if(norm_remote_cmd->PCKeyBoard.level_key_V && !norm_remote_cmd_last->PCKeyBoard.level_key_V){
// 			// 		HAL_NVIC_SystemReset();
// 			// 	}

// 			// X: 开关吊射模式 (sniper)
// 				if(norm_remote_cmd->PCKeyBoard.level_key_X && !norm_remote_cmd_last->PCKeyBoard.level_key_X)
// 				{
// 					gimbalControl.GimbalTargetInput.yaw_recoil_compensation_d = 0;
// 					robot_state->sniper ^= 1;
// 				}

//             // C: sniper_on下追逐一次yaw；sniper_off下切换上坡模式(CLIMB)
// 				if(norm_remote_cmd->PCKeyBoard.level_key_C && !norm_remote_cmd_last->PCKeyBoard.level_key_C)
// 				{
// 					if(robot_state->sniper == SNIPER_ON)
// 						robot_state->aim_mode = 1;
// 					else
// 					{
// 						if(robot_state->joint_mode == ROBOT_JOINT_MODE_CLIMB)
// 							robot_state->joint_mode = ROBOT_JOINT_MODE_NORMAL;
// 						else
// 							robot_state->joint_mode = ROBOT_JOINT_MODE_CLIMB;
// 					}
// 				}

// 			// F: 摩擦轮开关 (fric_mode)
// 				if(norm_remote_cmd->PCKeyBoard.level_key_F && !norm_remote_cmd_last->PCKeyBoard.level_key_F)
// 					robot_state->fric_mode++;

// 			// E: sniper下切换鼠标锁定，禁用鼠标角度输入
// 				if(norm_remote_cmd->PCKeyBoard.level_key_E && !norm_remote_cmd_last->PCKeyBoard.level_key_E)
// 					robot_state->mouse_fix ^= 1;

// 			// Q: 切换底盘模式（跟随 ↔ 陀螺旋转），仅sniper_off下生效
// 				if(robot_state->sniper == SNIPER_OFF && norm_remote_cmd->PCKeyBoard.level_key_Q && !norm_remote_cmd_last->PCKeyBoard.level_key_Q)
// 				{
// 					if(robot_state->chassis_mode == CHASSIS_FOLLOW)
// 						robot_state->chassis_mode = CHASSIS_REVOLVE;
// 					else if(robot_state->chassis_mode == CHASSIS_REVOLVE)
// 						robot_state->chassis_mode = CHASSIS_FOLLOW;
// 				}

// 			// G: 一键掉头 (FOLLOW_BACK)，再按一次换回来
// 				if(norm_remote_cmd->PCKeyBoard.level_key_G && !norm_remote_cmd_last->PCKeyBoard.level_key_G)
// 				{
// 					if(robot_state->chassis_mode == CHASSIS_FOLLOW_BACK)
// 						robot_state->chassis_mode = CHASSIS_FOLLOW;
// 					else
// 						robot_state->chassis_mode = CHASSIS_FOLLOW_BACK;
// 				}


// 			// V: 预备上台阶 (PRE_STAIR -> STAIR_UP)，再按一次回到普通模式
// 				if(norm_remote_cmd->PCKeyBoard.level_key_V && !norm_remote_cmd_last->PCKeyBoard.level_key_V)
// 				{
// 					if(robot_state->stand_mode == ROBOT_STAND_MODE_NORMAL)
// 						robot_state->stand_mode = ROBOT_STAND_MODE_PRE_STAIR;
// 					else if(robot_state->stand_mode == ROBOT_STAND_MODE_PRE_STAIR || robot_state->stand_mode == ROBOT_STAND_MODE_STAIR_UP)
// 						robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
// 				}

// 			// W: PRE_STAIR/STAIR_UP模式下履带旋转标志置1
// 				if(norm_remote_cmd->PCKeyBoard.level_key_W && (robot_state->stand_mode == ROBOT_STAND_MODE_PRE_STAIR || robot_state->stand_mode == ROBOT_STAND_MODE_STAIR_UP))
// 					crawler_rotate_flag = 1;
// 				else if(!norm_remote_cmd->PCKeyBoard.level_key_W)
// 					crawler_rotate_flag = 0;

// 			// B: sniper_off下切换下坡模式；sniper_on下切换画幅
// 				if(norm_remote_cmd->PCKeyBoard.level_key_B && !norm_remote_cmd_last->PCKeyBoard.level_key_B)
// 				{
// 					if(robot_state->sniper == SNIPER_ON)
// 					{
// 						/* sniper_on: B键切换画幅（暂不实现实质改变） */
// 					}
// 					else
// 					{
// 						if(robot_state->stand_mode == ROBOT_STAND_MODE_NORMAL)
// 							robot_state->stand_mode = ROBOT_STAND_MODE_PRE_DOWN_STAIR;
// 						else if(robot_state->stand_mode == ROBOT_STAND_MODE_PRE_DOWN_STAIR)
// 							robot_state->stand_mode = ROBOT_STAND_MODE_NORMAL;
// 					}
// 				}

// 			// Z: 切换摄影目标 (MID→UP→DOWN→MID)
// 				if(norm_remote_cmd->PCKeyBoard.level_key_Z && !norm_remote_cmd_last->PCKeyBoard.level_key_Z)
// 				{
// 					if(robot_state->cam_target == CAM_TARGET_MID)
// 						robot_state->cam_target = CAM_TARGET_UP;
// 					else if(robot_state->cam_target == CAM_TARGET_UP)
// 						robot_state->cam_target = CAM_TARGET_DOWN;
// 					else
// 						robot_state->cam_target = CAM_TARGET_MID;
// 				}

// 				if(robot_state->sniper == SNIPER_OFF)
// 					distance_check.distance_check_translate.angle = 0;

// 				static uint16_t shoot_wait = 1, //发弹间隔延迟计数
// 								wait_flag = 0;	//等待发弹结束标志位
// 			/*左键发射：摩擦轮需打开，热量需OK*/
// 			if(
// 				norm_remote_cmd->PCMouse.mouse_left
// 				&& robot_state->fric_mode == FRIC_ON
// 				&& (ext_game_robot_status.shooter_barrel_heat_limit - ext_power_heat_data.shooter_42mm_barrel_heat >= 100)
// 				&& shoot_wait==0)
// 				{
// 					robot_state->stir_mode = STIR_ANGLE_CONTROL;
// 					wait_flag = 0;
// 					shoot_wait += 60;//发弹延时
// 				}
// 				else{
// 					robot_state->stir_mode = STIR_LOCK;
// 				}
// 				if(shoot_wait)
// 					shoot_wait--;
// 				wait_see=(float)shoot_wait;
// 				/**/
// 				}
// 			separate_count = 0;
// 			break;
// 		case NORM_RC_SW_MID:
// 			robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;

// 			/* 测试: 左下右中 + CH4>0.1上升沿 → 触发一次拨盘堵转恢复(反转40度→回预置位) */
// 			{
// 				static uint8_t test_ch4_was_high = 0;
// 				if (norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_DOWN)
// 				{
// 					if (norm_remote_cmd->RelativeCH.ch4 > 0.1f && !test_ch4_was_high)
// 					{
// 						stall_count = 610;  /* >=600触发堵转判定,预留10次自减余量 */
// 					}
// 					test_ch4_was_high = (norm_remote_cmd->RelativeCH.ch4 > 0.1f);
// 				}
// 				else
// 				{
// 					test_ch4_was_high = 0;
// 				}
// 			}

// 		//摩擦轮未开状态向下拨两次进入分离状态 没有删除
// 			if(robot_state->fric_mode == FRIC_OFF && normRemoteCmdLast.Switch.switch_L1 != NORM_RC_SW_DOWN && norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_DOWN)
// 				separate_count++;
// 			if(separate_count > 2)
// 				separate_count = 0;
// 			if((norm_remote_cmd->RelativeCH.ch4) > 0.1){
// 				// robot_state->chassis_mode = CHASSIS_REVOLVE;
// 				}
// 			else if(separate_count == 2)
// 				robot_state->chassis_mode = CHASSIS_FOLLOW;
// 			else
// 				robot_state->chassis_mode = CHASSIS_FOLLOW;
		
// 			/*拨轮控制跟随方向：>0.1正向跟随，<-0.1反向180度跟随 */
// 			if(norm_remote_cmd->RelativeCH.ch4 > 0.1){
// 				robot_state->chassis_mode = CHASSIS_FOLLOW;
// 			}
// 			else if(norm_remote_cmd->RelativeCH.ch4 < -0.1){
// 				robot_state->chassis_mode = CHASSIS_FOLLOW_BACK;
// 			}
// 			else {
// 				robot_state->auto_slope = 0;
// 			}
// 			//遥控器左拨下 脉冲触发
// 			if(normRemoteCmdLast.Switch.switch_L1 != NORM_RC_SW_UP && norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP)
// 				robot_state->fric_mode++;
			
// 			if(_shootControl->ShootEstimate.stir_block_flag)
// 					robot_state->stir_mode = STIR_REVERSE;
				
// 			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_DOWN && robot_state->fric_mode != FRIC_OFF)
// 			{
// 				robot_state->stir_mode = STIR_ANGLE_CONTROL;
// 			}			
// 			else
// 				robot_state->stir_mode = STIR_LOCK;
			
// 			//robot_state->fric_mode=1;这个除了我不要取消注释
// 			//他妈的记得注释
// 			/*右拨中退出吊射模式*/
// 			robot_state->sniper=SNIPER_OFF;
// 			robot_state->follow=FOLLOW_OFF ;
// 		break;
		
// 		case NORM_RC_SW_DOWN:
// 			//给算法调试的时候不要右播下吊射模式
// 			/*右下进入遥控器吊射模式*/
// 			robot_state->sniper=SNIPER_ON;
// 			robot_state->ctrl_terminal = CONTROL_FROM_REMOTE;
// 		//死亡时进入保护状态
	

// 		/*调试用，在遥控器右下也可以用遥控器打弹*/
// 		if(normRemoteCmdLast.Switch.switch_L1 != NORM_RC_SW_UP && norm_remote_cmd->Switch.switch_L1 == NORM_RC_SW_UP)
// 				robot_state->fric_mode++;
			
// 			if(_shootControl->ShootEstimate.stir_block_flag)
// 					robot_state->stir_mode = STIR_REVERSE;
			
// 		//调试用，待会取消回来
// 		int mardio;
// 			if(normRemoteCmdLast.Switch.switch_L1 == NORM_RC_SW_DOWN && robot_state->fric_mode != FRIC_OFF)
// 			{
// 				robot_state->stir_mode = STIR_ANGLE_CONTROL;
// 			}			
// 			else
// 				robot_state->stir_mode = STIR_LOCK;
		
// 			/*右拨下开镜*/
// 				robot_state->sniper=SNIPER_ON;
// 			/*遥控器拨轮ch4切换世界坐标系yaw目标：>0.1使能上板yaw目标，<-0.1走原遥控逻辑*/
// 			if((norm_remote_cmd->RelativeCH.ch4) > 0.1){
// 				robot_state->world_enable = WORLD_ENABLE_ON;
// 			}
// 			if((norm_remote_cmd->RelativeCH.ch4) < -0.1){
// 				robot_state->world_enable = WORLD_ENABLE_OFF;
// 		}	
// 	}
// 	}/*这里是在switch之外*/
// //	if(robot_state->follow!=lastRobotState.follow)	
// //			{
// //				follow_leap_flag=1;
// //			}
// }

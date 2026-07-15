#include "stirControl.h"
#include "general_task_include.h"

/* 全局实例 */
ShootControl shootControl = {0};
const ShootControl* _shootControl = &shootControl;

/* 模块级变量 */
uint16_t stall_count = 0;
uint8_t  stir_stall_recovery_state = 0;
uint8_t  stir_flag = 0;
uint8_t  stir_two_step_phase = 0;   /* 二段拨弹: 0=空闲, 1=第一段30°完成等待300ms, 2=第二段30°完成 */
uint16_t stir_delay_counter = 0;    /* 二段拨弹延时计数 (30 = 300ms @10ms/周期) */

/* 射击/摩擦轮相关变量 — 从 robot_control_task.c 搬迁 */
float targetspeed[30] = {0};
float predict_speed0;
float mardio_speed = 15.75;
int16_t fric_speed_left_target, fric_speed_right_target, fric_speed_up_target;
int16_t fric_speed_left_target1, fric_speed_right_target1, fric_speed_up_target1;
float current_fric_speed = 4580;
float default_fric_speed = 3615;
float deltaspeed;
float emergesee;
float temp_angle1;
float temp_angle2;
int stir_cnt = 0;
int delay_cnt = 0;
int wait_cnt = 0;
int during_cnt = 0;

leastSquareLinear bulletSpeedAdaptation = {
    .x = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30},
    .count = 0
};

uint16_t CRC16_Modbus(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];   // 与当前字节异或

        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
uint8_t frame[8];

void ShootInputUpdate(void)
{
/*----------------------------------------------------stir------------------------------------------------------------*/
	/* 堵转恢复状态机: 0=空闲, 1=反转20度中, 2=回预置位中 */
	static float    stall_preset_target_d = 0.0f;  /* 堵转前的预置位目标 */
	static float    stall_reverse_target_d = 0.0f;  /* 反转20度后的目标 */
	static uint8_t  last_stir_block = 0;
	const float     stall_reverse_angle_d = 20.0f;   /* 反转角度(度) */
	const float     stall_arrive_tolerance_d = 4.0f; /* 到位容差(度) */

	/* 堵转上升沿: 中止二段拨弹, 保存预置位, 清除堵转标志, 开始反转20度 */
	if (shootControl.ShootEstimate.stir_block_flag == 1 && last_stir_block == 0 && stir_stall_recovery_state == 0)
	{
		stir_two_step_phase = 0;  /* 中止当前二段, 恢复完重新来过 */
		stir_flag = 0;
		stir_delay_counter = 0;

		stall_preset_target_d = shootControl.ShootTargetInput.stir_all_target_pos_d;
		stall_reverse_target_d = shootControl.ShootEstimate.stir_all_angle_d + stall_reverse_angle_d;
		shootControl.ShootTargetInput.stir_all_target_pos_d = stall_reverse_target_d;
		stall_count = 0;
		shootControl.ShootEstimate.stir_block_flag = 0;
		stir_stall_recovery_state = 1;  /* 反转中 */
	}
	last_stir_block = shootControl.ShootEstimate.stir_block_flag;

	/* 堵转恢复状态机处理 */
	switch (stir_stall_recovery_state)
	{
	case 1:  /* 反转中: 等待到达反转目标, 若中途再次堵转则放弃反转直接回预置位 */
		if (shootControl.ShootEstimate.stir_block_flag == 1)
		{
			/* 反转途中再次堵转: 放弃反转20度, 立即切回预置位 */
			stall_count = 0;
			shootControl.ShootEstimate.stir_block_flag = 0;
			shootControl.ShootTargetInput.stir_all_target_pos_d = stall_preset_target_d;
			stir_stall_recovery_state = 2;  /* 直接跳到回位中 */
		}
		else if (fabs(shootControl.ShootEstimate.stir_all_angle_d - stall_reverse_target_d) < stall_arrive_tolerance_d)
		{
			/* 反转到位, 设置目标为堵转前的预置位 */
			shootControl.ShootTargetInput.stir_all_target_pos_d = stall_preset_target_d;
			stir_stall_recovery_state = 2;  /* 回位中 */
		}
		break;

	case 2:  /* 回位中: 等待回到预置位, 若堵转则放弃回位就地对齐 */
		if (shootControl.ShootEstimate.stir_block_flag == 1)
		{
			/* 回位途中堵转: 放弃回位, 就地重新对齐六等分点, 结束恢复 */
			stall_count = 0;
			shootControl.ShootEstimate.stir_block_flag = 0;
			stir_stall_recovery_state = 0;  /* 空闲 */
			StirTargetAngleSet();
		}
		else if (fabs(shootControl.ShootEstimate.stir_all_angle_d - stall_preset_target_d) < stall_arrive_tolerance_d)
		{
			/* 回到预置位, 恢复正常, 重新对齐六等分点 */
			stir_stall_recovery_state = 0;  /* 空闲 */
			StirTargetAngleSet();
		}
		break;

	case 0:  /* 空闲 */
	default:
		break;
	}

	uint8_t in_stall_recovery = (stir_stall_recovery_state != 0);

	/* ===== 二段拨弹状态机 (20° + 300ms延时 + 40°) =====
	 * Phase 0: 空闲, 等待 stir_mode=ANGLE_CONTROL + 拨盘到位
	 * Phase 1: 第一段20°到位 → 延时300ms → 发出第二段40°
	 * Phase 2: 二段完成, 等待 stir_mode=LOCK 复归 (防止连发)
	 * ===================================================== */
	switch (stir_two_step_phase)
	{
	case 0:  /* 空闲 — 等待发射触发 */
		if (pDecisionAO->stir_mode == STIR_ANGLE_CONTROL
			&& fabs(shootControl.ShootTargetInput.stir_all_target_pos_d - shootControl.ShootEstimate.stir_all_angle_d) < 5.0f
			&& shootControl.ShootEstimate.stir_block_flag == 0
			&& !in_stall_recovery)
		{
			shootControl.ShootTargetInput.stir_all_target_pos_d -= 20.0f;  /* 第一段: 20° */
			stir_two_step_phase = 1;
			stir_delay_counter = 0;
			stir_flag = 1;

			shootControl.ShootEstimate.shoot_count++;
			extern void xvni_42_heart_da();
			xvni_42_heart_da();
			stall_count = 0;
			shootControl.ShootEstimate.stir_block_flag = 0;
		}
		break;

	case 1:  /* 第一段到位 → 延时300ms → 第二段40° */
		if (fabs(shootControl.ShootTargetInput.stir_all_target_pos_d - shootControl.ShootEstimate.stir_all_angle_d) < 5.0f)
		{
			if (++stir_delay_counter >= 30)  /* 30 × 10ms = 300ms */
			{
				shootControl.ShootTargetInput.stir_all_target_pos_d -= 40.0f;  /* 第二段: 40° */
				stir_two_step_phase = 2;
				stir_delay_counter = 0;
			}
		}
		else
		{
			stir_delay_counter = 0;  /* 未到位, 清零延时重等 */
		}
		break;

	case 2:  /* 二段完成 — 等待 stir_mode=LOCK 复归, 防止连发 */
		if (pDecisionAO->stir_mode == STIR_LOCK)
		{
			stir_two_step_phase = 0;
			stir_flag = 0;
		}
		break;
	}

	shootControl.ShootTargetInput.stir_target_pos_rad = shootControl.ShootTargetInput.stir_target_pos / 180.0f * PI;
	shootControl.ShootTargetInput.stir_all_target_pos_rad =shootControl.ShootTargetInput.stir_all_target_pos_d/ 180.0f * PI;
	shootControl.ShootTargetInput.shoot_flag = (pDecisionAO->stir_mode != STIR_LOCK) ? 1 : 0;

	/* 堵转恢复期间不锁电机, 确保电机可反转和回位 */
	if(pDecisionAO->ctrl_terminal == CONTROL_STOP)
	{
		lock_motor(&hfdcan1,GMJ4310MOTOR_ID);
	}
	else if (in_stall_recovery)
	{
		/* 堵转恢复期间确保电机处于运行状态 */
		if (_stirMotorRec->state == 0)
			start_motor(&hfdcan1, GMJ4310MOTOR_ID);
	}
	else if (shootControl.ShootEstimate.stir_block_flag == 1 && _stirMotorRec->state == 1)
	{
		lock_motor(&hfdcan1,GMJ4310MOTOR_ID);
	}
	else if(pDecisionAO->ctrl_terminal != CONTROL_STOP && shootControl.ShootEstimate.stir_block_flag == 0 && _stirMotorRec->state == 0)
	{
		start_motor(&hfdcan1, GMJ4310MOTOR_ID);
	}

	GetStirRealAngle();
}

void ShootEstimateUpdate(void)
{
	/*拨盘电机使能失能认定*/
	static uint8_t disbuf = 0;
	static uint8_t enablebuf = 0;
	static uint16_t stir_state_detect_counter = 0;
	stir_state_detect_counter++;

	/*新堵转检测*/
	static uint8_t reverse_count = 0;//延迟恢复计数
	//static uint16_t stall_count = 0;//堵转时间计数
	//if(fabs(_stirMotorRec->vel_radps) < STIR_CAUTION_SPEED && fabs(_stirMotorRec->toq) > 5.0f)
	if(fabs(_stirMotorRec->vel_radps) < STIR_CAUTION_SPEED && fabs(_stirMotorRec->toq) > 8.0f)
		stall_count+=10;
	else if(stall_count>0)
		stall_count--;

	if(stall_count>=500)//400
		shootControl.ShootEstimate.stir_block_flag = 1;      //1
	if(stall_count==0)
		shootControl.ShootEstimate.stir_block_flag = 0;
	/*新拨盘自动预制 退保护就转拨盘*/

//	/*拨盘自动预置*/
	static uint8_t last_fric_state, cur_fric_state, last_robot_state, cur_robot_state;
	cur_fric_state = pDecisionAO->fric_mode;
	cur_robot_state = pDecisionAO->ctrl_terminal;
//	if((last_fric_state == 0 && cur_fric_state) || (last_robot_state == 0 && cur_robot_state))
	if(last_robot_state == 0 && cur_robot_state && _stirMotorRec->frame_counter)
	{
		StirTargetAngleSet();
	}
	last_fric_state = cur_fric_state;
	last_robot_state = cur_robot_state;

	/*10s无操作自动校准*/
	static uint16_t calibration_count = 0;
	if(_stirMotorRec->vel_radps < STIR_CAUTION_SPEED && pDecisionAO->ctrl_terminal != CONTROL_STOP && !shootControl.ShootEstimate.stir_reset_flag)
	{
		calibration_count++;
		if(calibration_count % 1000 == 0)
			shootControl.ShootEstimate.stir_real_angle = shootControl.ShootTargetInput.stir_target_pos;//?
	}
	else
		calibration_count = 0;

	static uint16_t last_count = 0, last_last_count = 0, cur_count, time_count = 0;
	time_count++;
	cur_count = _stirMotorRec->frame_counter;
	if(time_count % 100 == 0)
		{/*连这三个周期不变的话，认定为拨盘下电*/
		if(last_last_count == last_count && last_count == cur_count && !shootControl.ShootEstimate.stir_reset_flag)
		{
			shootControl.ShootEstimate.stir_real_angle = 0;
			shootControl.ShootEstimate.stir_real_angle_d = 0;
			shootControl.ShootEstimate.stir_angle_last = 0;
			shootControl.ShootEstimate.stir_angle_cur = 0;
			shootControl.ShootEstimate.stir_real_angle_rad = 0;
			shootControl.ShootEstimate.stir_reset_flag++;//生成重置信号
		}
		last_count = cur_count;
		last_last_count = last_count;
	}
//		//获取拨盘转动总角度
	GetStirRealAngle();
	if(!shootControl.ShootEstimate.stir_reset_flag && _stirMotorRec->frame_counter)
	{
		GetStirRealAngle();//
	}
	if(shootControl.ShootEstimate.stir_reset_flag && ext_game_robot_status.power_management_shooter_output && cur_count != last_count)
	{/*如果有重置flag，并且*/
		shootControl.ShootEstimate.stir_reset_flag = 0;
		GetStirRealAngle();
		StirTargetAngleSet();
	}
}
void GetStirRealAngle(void)
{
	shootControl.ShootEstimate.stir_angle_cur = _stirMotorRec->pos_d;
	shootControl.ShootEstimate.stir_real_angle_d = shootControl.ShootEstimate.stir_angle_cur - shootControl.ShootEstimate.stir_angle_last;

//	if(shootControl.ShootEstimate.stir_real_angle_d > 720)
//		shootControl.ShootEstimate.stir_real_angle_d -= 1440.0f;
//	if(shootControl.ShootEstimate.stir_real_angle_d < -720)
//		shootControl.ShootEstimate.stir_real_angle_d += 1440.0f;
	shootControl.ShootEstimate.stir_real_angle += shootControl.ShootEstimate.stir_real_angle_d;
	/*圈数记录*/
//	if(shootControl.ShootEstimate.stir_angle_cur<-160 && shootControl.ShootEstimate.stir_angle_last>+160)
//		shootControl.ShootEstimate.quan_shu_r++;
//	if(shootControl.ShootEstimate.stir_angle_cur>+160 && shootControl.ShootEstimate.stir_angle_last<-160)
//		shootControl.ShootEstimate.quan_shu_r--;
	/*圈数记录：先更新quan_shu_r，再计算含圈数的stir_all_angle_d，确保堵转恢复到达判断在同一坐标系 */
	if(shootControl.ShootEstimate.stir_angle_cur<-DM_MOTO_MAX_ENCODE_D	+100 && shootControl.ShootEstimate.stir_angle_last>DM_MOTO_MAX_ENCODE_D	-100)
		shootControl.ShootEstimate.quan_shu_r++;
	if(shootControl.ShootEstimate.stir_angle_cur>+DM_MOTO_MAX_ENCODE_D 	-100 && shootControl.ShootEstimate.stir_angle_last<-DM_MOTO_MAX_ENCODE_D +100)
		shootControl.ShootEstimate.quan_shu_r--;
	/*真实角度记录：包含圈数，与stir_all_target_pos_d在同一坐标系 */
	shootControl.ShootEstimate.stir_all_angle_d=(_stirMotorRec->pos_d) + (shootControl.ShootEstimate.quan_shu_r * DM_MOTO_MAX_ENCODE_D);


	shootControl.ShootEstimate.stir_angle_last = shootControl.ShootEstimate.stir_angle_cur;
	shootControl.ShootEstimate.stir_real_angle_rad = shootControl.ShootEstimate.stir_real_angle / 180.0f * 3.14f;
}
float yes_60_angle_d,temp_angle_d;
float compale_angle_d;
float dealt_d;
float temp_select_angle_d=50;//正数，如果是30就是找最近点,调小就是更容易往前
/**
 * @brief 将角度对齐到最近的六等分点（带偏移）
 * @param current_angle 输入角度（单位：度）
 * @param offset 整体偏移量
 * @return 对齐后的角度（0.0f ~ 360.0f）
 */
/*2025.6.29:注意，有出现，退保护第一次反向堵转的情况，这是因为最近的六等分点在后方，被一颗大胆玩卡到了，所以确实算是堵转；这种情况应该往前转
解决方案：不一定是最近的六等分点，这个和预置位有关系*/
void StirTargetAngleSet(void)
{
		temp_angle_d = _stirMotorRec->pos_d;
		temp_angle_d -= shootControl.ShootMotorControl.stir_preset_angle;//预置位
		dealt_d = fmod(temp_angle_d,60);
		/*对fmod的特性进行处理，若a<b，fmod会返回小的值*/
	/*你希望它尽量往前*/

		if(dealt_d>temp_select_angle_d)
			dealt_d = (dealt_d-60);
		if(dealt_d<-(60-temp_select_angle_d))
			dealt_d = (dealt_d+60);
		/*角度核心*/
		shootControl.ShootTargetInput.stir_all_target_pos_d=(_stirMotorRec->pos_d-dealt_d)+(shootControl.ShootEstimate.quan_shu_r*DM_MOTO_MAX_ENCODE_D);
		shootControl.ShootTargetInput.stir_all_target_pos_rad = shootControl.ShootTargetInput.stir_all_target_pos_d/180.0f * PI;
}
/**
 * @brief 底盘闭环控制
 */

void ShootControlUpdate(void)
{
	if(shootControl.ShootEstimate.stir_block_flag && stir_stall_recovery_state == 0)
		shootControl.ShootTargetInput.stir_target_vol = 0;//堵转恢复期间允许电机运动
	else
	{
		if(gimbalControl.GimbalEstimate.pitch_angle_d<0){
		shootControl.ShootTargetInput.stir_target_vol = STIR_MAX_SPEED_LOW;
		}
		else
		{
			shootControl.ShootTargetInput.stir_target_vol = STIR_MAX_SPEED;
		}
	}
	/*一下有保护*/
	if(CONTROL_STOP == pDecisionAO->ctrl_terminal)
	{
		shootControl.ShootEstimate.stir_enableflag_desire = DISABLE; // 期望失能
	}

	#if defined SHOOT_OFF
		shootControl.ShootMotorControl.fric_target_output[LEFT] = shootControl.ShootMotorControl.fric_target_output[RIGHT] = 0;
		shootControl.ShootEstimate.stir_enableflag_desire = DISABLE; // 期望失能
	#endif
}

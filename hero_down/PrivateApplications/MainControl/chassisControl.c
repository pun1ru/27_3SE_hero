#include "chassisControl.h"
#include "general_task_include.h"

/* 全局实例 */
ChassisControl chassisControl = {0};
const ChassisControl* _chassisControl = &chassisControl;

/* 模块级变量 */
int temp_yaw = 0, last_temp_yaw = 0;
SmoothFilter MouseFilterX = {0};
SmoothFilter MouseFilterY = {0};
AverageFilter PowerFilter = {0};

/* general_task_include.h 里没有的外部引用 — 需要单独 extern */
DJIGMotorRec yawMotorRec = {0}; /* yaw DM已搬迁至上板，下板不再有yaw DJI电机，保持零值 */
extern Pose                    gimbalPose;

static float chassis_calculate_prior_power(void);

void ChassisDecisionInitialize(void)
{
    PIDInitialize(&chassisControl.ChassisFollowControl.follow_speed_need_pid, -0.02, 0, 0.008, 0, 3);
    PIDInitialize(&chassisControl.GimbalCoordinateInput.speed_x_compensate_pid, 0.1, 0.1, 0, 1, 3);
    PIDInitialize(&chassisControl.GimbalCoordinateInput.speed_y_compensate_pid, 0.1, 0.1, 0, 1, 3);
}

void ChassisControlInitialize(void)
{
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[LF]), 8, 0, 2.0, 0, M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[RF]), 8, 0, 2.0, 0, M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[RB]), 7, 0, 1.5, 0, M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[LB]), 7, 0, 1.5, 0, M3508_MAX_OUTPUT_CURRENT);
    AverageFilterInitialize(&PowerFilter);
}

typedef struct
{
    uint8_t is_input;
    uint8_t is_x_input;
    uint8_t is_y_input;
    float max_linear_speed_x_mps;
    float max_linear_speed_y_mps;
    float max_speed_error_x_mps;
    float max_speed_error_y_mps;
    float delta_angle_sin;
    float delta_angle_cos;
} ChassisTranslationContext;

static void chassis_update_remote_translation(ChassisTranslationContext* context)
{
    if(fabs(_normRemoteCmd->RelativeCH.ch0) < 0.1 && fabs(_normRemoteCmd->RelativeCH.ch1) < 0.1
       && fabs(_normRemoteCmd->RelativeCH.ch4) < 0.1)
        context->is_input = 0;

    if(context->is_input)
    {
        if(fabs(chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps)
           < context->max_speed_error_x_mps)
            chassisControl.GimbalCoordinateInput.speed_x_mps += _normRemoteCmd->RelativeCH.ch0 * 0.015f;

        if(fabs(chassisControl.GimbalCoordinateInput.speed_y_mps - chassisControl.ChassisEstimate.speed_y_mps)
           < context->max_speed_error_y_mps)
            chassisControl.GimbalCoordinateInput.speed_y_mps += _normRemoteCmd->RelativeCH.ch1 * 0.015f
                                                                + _robotState->auto_slope * 0.015f
                                                                + _robotState->auto_slope * 0.07f
                                                                      * (chassisControl.ChassisEstimate.speed_y_mps - 0.25)
                                                                      * (chassisControl.ChassisEstimate.speed_y_mps > 0.25
                                                                         && chassisControl.ChassisEstimate.speed_y_mps <= 4.0);

        if(_normRemoteCmd->RelativeCH.ch0 * chassisControl.ChassisEstimate.speed_x_mps < 0)
        {
            if(chassisControl.GimbalCoordinateInput.speed_x_mps * _normRemoteCmd->RelativeCH.ch0 < 0)
                chassisControl.GimbalCoordinateInput.speed_x_mps = 0;
            chassisControl.GimbalCoordinateInput.speed_x_mps += _normRemoteCmd->RelativeCH.ch0 * 0.01f;
        }

        if(_normRemoteCmd->RelativeCH.ch1 * chassisControl.ChassisEstimate.speed_y_mps < 0)
        {
            if(chassisControl.GimbalCoordinateInput.speed_y_mps * _normRemoteCmd->RelativeCH.ch1 < 0)
                chassisControl.GimbalCoordinateInput.speed_y_mps = 0;
            chassisControl.GimbalCoordinateInput.speed_y_mps += _normRemoteCmd->RelativeCH.ch1 * 0.01f;
        }

        chassisControl.GimbalCoordinateInput.speed_x_mps = AbsLimiter(
            chassisControl.GimbalCoordinateInput.speed_x_mps,
            context->max_linear_speed_x_mps * fabs(_normRemoteCmd->RelativeCH.ch0));
        chassisControl.GimbalCoordinateInput.speed_y_mps = AbsLimiter(
            chassisControl.GimbalCoordinateInput.speed_y_mps,
            context->max_linear_speed_y_mps * (fabs(_normRemoteCmd->RelativeCH.ch1) + _robotState->auto_slope));
    }
    else
    {
        chassisControl.GimbalCoordinateInput.speed_x_mps = chassisControl.ChassisEstimate.speed_x_mps;
        chassisControl.GimbalCoordinateInput.speed_y_mps = chassisControl.ChassisEstimate.speed_y_mps;
    }
}

static void chassis_update_pc_translation(ChassisTranslationContext* context)
{
    if(!_normRemoteCmd->PCKeyBoard.level_key_A && !_normRemoteCmd->PCKeyBoard.level_key_D)
        context->is_x_input = 0;
    if(!_normRemoteCmd->PCKeyBoard.level_key_W && !_normRemoteCmd->PCKeyBoard.level_key_S
       && !_normRemoteCmd->PCKeyBoard.level_key_E)
        context->is_y_input = 0;
    if(context->is_x_input == 0 && context->is_y_input == 0)
        context->is_input = 0;

    if(context->is_x_input)
    {
        int8_t input_x = _normRemoteCmd->PCKeyBoard.level_key_D - _normRemoteCmd->PCKeyBoard.level_key_A;
        if(fabs(chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps)
           < context->max_speed_error_x_mps)
            chassisControl.GimbalCoordinateInput.speed_x_mps += input_x * 0.01f;

        if(input_x * chassisControl.ChassisEstimate.speed_x_mps < 0)
        {
            if(chassisControl.GimbalCoordinateInput.speed_x_mps * input_x < 0)
                chassisControl.GimbalCoordinateInput.speed_x_mps = 0;
            chassisControl.GimbalCoordinateInput.speed_x_mps += input_x * 0.01f;
        }
        chassisControl.GimbalCoordinateInput.speed_x_mps = AbsLimiter(
            chassisControl.GimbalCoordinateInput.speed_x_mps, context->max_linear_speed_x_mps);
    }
    else
    {
        chassisControl.GimbalCoordinateInput.speed_x_mps = chassisControl.ChassisEstimate.speed_x_mps;
    }

    if(context->is_y_input)
    {
        int8_t input_y = _normRemoteCmd->PCKeyBoard.level_key_W - _normRemoteCmd->PCKeyBoard.level_key_S;
        if(fabs(chassisControl.GimbalCoordinateInput.speed_y_mps - chassisControl.ChassisEstimate.speed_y_mps)
           < context->max_speed_error_y_mps)
        {
            if(_robotState->auto_slope == 1)
            {
                chassisControl.GimbalCoordinateInput.speed_y_mps += _robotState->auto_slope * 0.015f
                                                                    + _robotState->auto_slope * 0.07f
                                                                          * (chassisControl.ChassisEstimate.speed_y_mps - 0.25)
                                                                          * (chassisControl.ChassisEstimate.speed_y_mps > 0.25
                                                                             && chassisControl.ChassisEstimate.speed_y_mps <= 4.0);
            }
            else
            {
                chassisControl.GimbalCoordinateInput.speed_y_mps += input_y * 0.01f
                                                                    + _robotState->auto_slope * 0.005f
                                                                    + _robotState->auto_slope * 0.005f
                                                                          * (chassisControl.ChassisEstimate.speed_y_mps - 0.65)
                                                                          * (chassisControl.ChassisEstimate.speed_y_mps > 0.65
                                                                             && chassisControl.ChassisEstimate.speed_y_mps <= 1.5)
                                                                    + _robotState->auto_slope * 0.0085f
                                                                          * (chassisControl.ChassisEstimate.speed_y_mps > 1.35);
            }
        }

        if(input_y * chassisControl.ChassisEstimate.speed_y_mps < 0)
        {
            if(chassisControl.GimbalCoordinateInput.speed_y_mps * input_y < 0)
                chassisControl.GimbalCoordinateInput.speed_y_mps = 0;
            chassisControl.GimbalCoordinateInput.speed_y_mps += input_y * 0.01f;
        }
        chassisControl.GimbalCoordinateInput.speed_y_mps = AbsLimiter(
            chassisControl.GimbalCoordinateInput.speed_y_mps, context->max_linear_speed_y_mps);
    }
    else
    {
        chassisControl.GimbalCoordinateInput.speed_y_mps = chassisControl.ChassisEstimate.speed_y_mps;
    }
}

static void chassis_update_translation_source(ChassisTranslationContext* context)
{
    switch(pDecisionAO->ctrl_terminal)
    {
        case CONTROL_STOP:
            chassisControl.GimbalCoordinateInput.speed_x_mps = 0;
            chassisControl.GimbalCoordinateInput.speed_y_mps = 0;
            chassisControl.ChassisFollowControl.revolve_return_flag = 0;
            break;

        case CONTROL_FROM_REMOTE:
            chassis_update_remote_translation(context);
            break;

        case CONTROL_FROM_PC:
            chassis_update_pc_translation(context);
            break;
    }
}

static void chassis_finalize_translation_input(const ChassisTranslationContext* context)
{
    float norm = context->max_linear_speed_y_mps
                 / sqrt(chassisControl.GimbalCoordinateInput.speed_x_mps
                            * chassisControl.GimbalCoordinateInput.speed_x_mps
                        + chassisControl.GimbalCoordinateInput.speed_y_mps
                            * chassisControl.GimbalCoordinateInput.speed_y_mps);
    if(norm <= 1)
    {
        chassisControl.GimbalCoordinateInput.speed_x_mps *= norm;
        chassisControl.GimbalCoordinateInput.speed_y_mps *= norm;
    }

    chassisControl.GimbalCoordinateInput.speed_x_mps *= context->is_input * context->is_x_input;
    chassisControl.GimbalCoordinateInput.speed_y_mps *= context->is_input * context->is_y_input;
    PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid));
    PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_y_compensate_pid));

    if(chassisControl.GimbalCoordinateInput.speed_x_mps == 0)
    {
        chassisControl.GimbalCoordinateInput.speed_x_mps += PIDUpdate(
            &(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid),
            chassisControl.GimbalCoordinateInput.speed_x_mps - chassisControl.ChassisEstimate.speed_x_mps);
    }
    else
    {
        PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid));
    }

#ifdef CENTRIFUGE_REVOLVE
    if(pDecisionAO->chassis_mode == CHASSIS_REVOLVE)
    {
        chassisControl.GimbalCoordinateInput.speed_y_mps += 0.1f * context->delta_angle_cos;
        chassisControl.GimbalCoordinateInput.speed_x_mps += 0.1f * context->delta_angle_sin;
    }
#endif

    chassisControl.ChassisCoordinateInput.speed_x_mps = chassisControl.GimbalCoordinateInput.speed_x_mps
                                                          * context->delta_angle_cos
                                                       + chassisControl.GimbalCoordinateInput.speed_y_mps
                                                          * context->delta_angle_sin;
    chassisControl.ChassisCoordinateInput.speed_y_mps = -chassisControl.GimbalCoordinateInput.speed_x_mps
                                                          * context->delta_angle_sin
                                                       + chassisControl.GimbalCoordinateInput.speed_y_mps
                                                          * context->delta_angle_cos;
}

static uint8_t chassis_update_translation_input(void)
{
    ChassisTranslationContext context = {
        .is_input = 1,
        .is_x_input = 1,
        .is_y_input = 1,
        .max_linear_speed_x_mps = 3,
        .max_linear_speed_y_mps = 3,
        .max_speed_error_x_mps = 0.04 / (0.7 * fabs(chassisControl.ChassisEstimate.speed_x_mps) + 0.5) + 0.15,
        .max_speed_error_y_mps = 0.07 / (0.7 * fabs(chassisControl.ChassisEstimate.speed_y_mps) + 0.5) + 0.22
                                 - (_robotState->auto_slope * 0.2
                                    * (chassisControl.ChassisEstimate.speed_y_mps > 1.95)),
    };

    arm_sin_cos_f32(-chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d,
                    &context.delta_angle_sin, &context.delta_angle_cos);
    chassis_update_translation_source(&context);
    chassis_finalize_translation_input(&context);
    return context.is_input;
}

static void chassis_update_rotation_limit(uint8_t is_input_flag)
{
	/*-------------------------------------------------------------revolve------------------------------------------------------------*/
	chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.6;

	//50w->0.65 	60w->0.75		70w->0.85	80w->0.95	100w->1.10  	120w->1.25
	if(ext_game_robot_status.chassis_power_limit >= 120)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 1.25f;
	else if(ext_game_robot_status.chassis_power_limit >= 100)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 1.10f;
	else if(ext_game_robot_status.chassis_power_limit >= 80)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.95f;
	else if(ext_game_robot_status.chassis_power_limit >= 70)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.85f;
	else if(ext_game_robot_status.chassis_power_limit >= 60)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.75f;
	else if(ext_game_robot_status.chassis_power_limit >= 50)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.65f;


	if(_normRemoteCmd->PCKeyBoard.level_key_SHIFT)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps += 0.3;

	if(is_input_flag)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps -= 0.05;

	if(_robotState->capacity_mode == NO_CAPACITY)
		chassisControl.GimbalCoordinateInput.max_revolve_speed_rps -= 0.05;

	//比赛模式下为盲道检录降陀螺转速
	#ifdef MATCH_MODE
		static uint16_t match_count = 0;
		if(CONTROL_FROM_REMOTE == pDecisionAO->ctrl_terminal && pDecisionAO->chassis_mode == CHASSIS_REVOLVE)
			match_count = 300;
		if(match_count > 0)
		{
			match_count--;
			chassisControl.GimbalCoordinateInput.max_revolve_speed_rps = 0.2f;
		}
	#endif
}

static void chassis_update_rotation_target(void)
{
	/*底盘跟随及自旋方向速度输入*/
	switch(pDecisionAO->chassis_mode)
	{
		case CHASSIS_FOLLOW:
		case CHASSIS_FOLLOW_BACK:
			//需要判断此时是刚从自旋变成跟随

			//正常的跟随
			if(chassisControl.ChassisFollowControl.revolve_return_flag == 0){

				if(_normRemoteCmd->PCMouse.mouse_right && _upperComputerComm->Receive.aiming_state == 0x33)
					chassisControl.ChassisCoordinateInput.speed_w_rps = PIDUpdate(&(chassisControl.ChassisFollowControl.follow_speed_need_pid),\
																			  chassisControl.ChassisEstimate.chassis_follow_angle_d\
																				+-0.5*(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d,-180,+180))/*新增陈宝群补偿项，注意符号*/\
																				);
				else
					chassisControl.ChassisCoordinateInput.speed_w_rps = PIDUpdate(&(chassisControl.ChassisFollowControl.follow_speed_need_pid),\
																			  chassisControl.ChassisEstimate.chassis_follow_angle_d\
																				+-0.5*(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d,-180,+180))/*新增陈宝群补偿项，注意符号*/\
																				);
																				}
			//刚从自旋变成跟随
			else
			{
				/* 切跟随瞬间：yaw目标对齐当前云台实际朝向，用正常跟随逻辑驱动回正 */
				gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;

				chassisControl.ChassisCoordinateInput.speed_w_rps = PIDUpdate(&(chassisControl.ChassisFollowControl.follow_speed_need_pid),\
																			  chassisControl.ChassisEstimate.chassis_follow_angle_d\
																				+-0.5*(AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d - gimbalControl.GimbalEstimate.yaw_angle_d,-180,+180)));

				if(fabs(AngleLimit(chassisControl.ChassisEstimate.chassis_follow_angle_d, -180, 180)) < 5.0f)
					chassisControl.ChassisFollowControl.revolve_return_flag = 0;
			}
		break;

		case CHASSIS_REVOLVE:
			if((CONTROL_FROM_REMOTE == pDecisionAO->ctrl_terminal || CONTROL_FROM_PC == pDecisionAO->ctrl_terminal) && DT7 == _normRemoteCmd->remote_source)
			{
				chassisControl.ChassisCoordinateInput.speed_w_rps = -chassisControl.GimbalCoordinateInput.max_revolve_speed_rps;//调试限速，赛场上不限速，你妈的记得删

				if(_normRemoteCmd->RelativeCH.ch4 < -0.1f){
					// chassisControl.ChassisCoordinateInput.speed_w_rps *= -1;
					}

				chassisControl.ChassisFollowControl.revolve_return_flag = Sign(chassisControl.ChassisEstimate.speed_w_rps);//回正方向
			}
		break;

		case CHASSIS_SEPARATE:
			chassisControl.ChassisCoordinateInput.speed_w_rps = 0;
		break;
	}
}

static void chassis_update_rotation_observer(void)
{
	/*如果少了轮子，事实上运动学解算也是相同的，
		但是有比较麻烦的力速摩擦等等关系。
		我直接当做黑箱处理然后PID补偿掉*/

	/*首先，我简单的想想，掉左边的轮子就是往左边转，所以要往右边补偿，否则反之，大概是前馈？*/
	//chassisControl.ChassisCoordinateInput.speed_w_rps +=

	/*或者直行的时候IMU角速度闭环？简单的P控制先 其中chassisControl.ChassisCoordinateInput.compensate_speed_w_dps是error项目*/
	//不对不应该是gimbal yaw
	chassisControl.ChassisEstimate.imu_yaw_dps=gimbalControl.GimbalEstimate.yaw_angular_velocity_dps-yawMotorRec.mechanical_speed_rpm*360/60;//观测器给出的
	chassisControl.ChassisCoordinateInput.compensate_speed_w_dps=(360*chassisControl.ChassisCoordinateInput.speed_w_rps-chassisControl.ChassisEstimate.imu_yaw_dps);//error项
	//chassisControl.ChassisCoordinateInput.speed_w_rps +=chassisControl.ChassisCoordinateInput.compensate_speed_w_dps/360;
	//以上两个方案应该选一个就行0 事实上三轮可以开，那个差角补偿稍微给大一点
}

static void chassis_update_rotation_input(uint8_t is_input_flag)
{
	chassis_update_rotation_limit(is_input_flag);
	chassis_update_rotation_target();
	chassis_update_rotation_observer();

	//自旋速度限幅
	chassisControl.ChassisCoordinateInput.speed_w_rps = limiter(chassisControl.ChassisCoordinateInput.speed_w_rps, chassisControl.GimbalCoordinateInput.max_revolve_speed_rps);
}

static void chassis_apply_input_guards(void)
{
	//保护赋值
	if(CONTROL_STOP == pDecisionAO->ctrl_terminal)
		chassisControl.ChassisCoordinateInput.speed_w_rps = 0;

	/*另外的状态机，在吊射模式下防止底盘跟随，速度给0，但是输出不给0，尽量保持静止
	后来发现可以和分离模式耦合*/
	if((pDecisionAO->sniper == SNIPER_ON)){
		chassisControl.ChassisCoordinateInput.speed_x_mps=0;
		chassisControl.ChassisCoordinateInput.speed_y_mps=0;
		chassisControl.ChassisCoordinateInput.speed_w_rps=0;
			PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_x_compensate_pid));
			PIDRefreshBuffer(&(chassisControl.GimbalCoordinateInput.speed_y_compensate_pid));
	}
}

void ChassisInputUpdate(void)
{
    uint8_t is_input_flag = chassis_update_translation_input();

    chassis_update_rotation_input(is_input_flag);
    chassis_apply_input_guards();
}

/**
 * @brief 底盘相关观测数据更新
 */
static void chassis_update_gimbal_relation(void)
{
		/* 底盘云台相对角度：取反即得（B2B 0x228 yaw = DM编码器°，offset已内置到上板计算） */
		extern volatile float gimbal_yaw_rx_d;
		chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d = -gimbal_yaw_rx_d;
	chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d = AngleLimit(chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d, -180, 180);
	/*跟随角度 = 云台与底盘的相对角，云台正前方时为0*/
	chassisControl.ChassisEstimate.chassis_follow_angle_d = chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d;														  //+ (- gimbalControl.GimbalMotorControl.yaw_angle_adrc.esf.e_1);
	/* CHASSIS_FOLLOW_BACK: 跟随方向反转180度，机体背向云台朝向 */
	if (pDecisionAO->chassis_mode == CHASSIS_FOLLOW_BACK)
	{
		chassisControl.ChassisEstimate.chassis_follow_angle_d += 180.0f;
	}

	static uint16_t shake_count = 0;
	//唉，为什么这里一点那里一点啊,我也懒得给你重构了，加史
	if(pDecisionAO->sniper == SNIPER_OFF)
	if(_normRemoteCmd->PCKeyBoard.level_key_CTRL)
		shake_count = 400;
	if (shake_count > 0)
	{
		shake_count--;
		switch (shake_count / 50)
		{
			case 8: case 7: case 5: case 3: case 1: chassisControl.ChassisEstimate.chassis_follow_angle_d += 45;break;
			default: chassisControl.ChassisEstimate.chassis_follow_angle_d -= 45;
		}
	}

	chassisControl.ChassisEstimate.chassis_follow_angle_d = AngleLimit(chassisControl.ChassisEstimate.chassis_follow_angle_d, -180, 180);
}

static void chassis_update_velocity_estimate(void)
{
	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
		chassisControl.ChassisEstimate.wheel_real_speed_mps[i] = -_chassisMotorRec[i].mechanical_speed_rpm * WHEEL_RPM_TO_WHEEL_MPS;

	/*根据底盘电机安装方向，逆运动学解算出当前底盘的真实速度*/
	chassisControl.ChassisEstimate.speed_w_rps = (chassisControl.ChassisEstimate.wheel_real_speed_mps[LF] + chassisControl.ChassisEstimate.wheel_real_speed_mps[RF]\
												 +chassisControl.ChassisEstimate.wheel_real_speed_mps[LB] + chassisControl.ChassisEstimate.wheel_real_speed_mps[RB])\
												 / 4.0f * WHEEL_MPS_TO_ROBOT_RPS;

	float chassis_coordinate_speed_x_mps = (chassisControl.ChassisEstimate.wheel_real_speed_mps[LF] + chassisControl.ChassisEstimate.wheel_real_speed_mps[RF]\
											 - chassisControl.ChassisEstimate.wheel_real_speed_mps[LB] - chassisControl.ChassisEstimate.wheel_real_speed_mps[RB])\
											 / 4.0f / 1.414f;

	float chassis_coordinate_speed_y_mps = (chassisControl.ChassisEstimate.wheel_real_speed_mps[LF] - chassisControl.ChassisEstimate.wheel_real_speed_mps[RF]\
											 + chassisControl.ChassisEstimate.wheel_real_speed_mps[LB] - chassisControl.ChassisEstimate.wheel_real_speed_mps[RB])\
											 / 4.0f / 1.414f;

	float delta_angle_cos, delta_angle_sin;
	arm_sin_cos_f32(chassisControl.ChassisEstimate.gimbal_to_chassis_delta_angle_d, &delta_angle_sin, &delta_angle_cos);
	chassisControl.ChassisEstimate.speed_x_mps = chassis_coordinate_speed_x_mps * delta_angle_cos \
											   - chassis_coordinate_speed_y_mps * delta_angle_sin;
	chassisControl.ChassisEstimate.speed_y_mps = chassis_coordinate_speed_x_mps * delta_angle_sin \
											   + chassis_coordinate_speed_y_mps * delta_angle_cos;
}

void ChassisEstimateUpdate(void)
{
    chassis_update_gimbal_relation();
    chassis_update_velocity_estimate();
}

/**
 * @brief 底盘闭环控制
 */

static void chassis_prepare_control_target(void)
{

	//实际需要的底盘速度为1.0*target + ratio*(target-real)，即引入反馈量，在原速度向量的基础上叠加反馈修正速度使得实际速度向量更快收敛到原目标速度
	chassisControl.ChassisRealNeedInput.speed_x_mps = chassisControl.ChassisCoordinateInput.speed_x_mps;

	chassisControl.ChassisRealNeedInput.speed_y_mps = chassisControl.ChassisCoordinateInput.speed_y_mps;

	chassisControl.ChassisRealNeedInput.speed_w_rps = chassisControl.ChassisCoordinateInput.speed_w_rps;
	if (pDecisionAO->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB )
	{
		if (fabs(_normRemoteCmd->RelativeCH.ch1) > 0.1f)
		{
			chassisControl.ChassisRealNeedInput.speed_y_mps = chassisControl.ChassisCoordinateInput.speed_y_mps;
		}
		else
		chassisControl.ChassisRealNeedInput.speed_y_mps = 0.0f;
		chassisControl.ChassisRealNeedInput.speed_x_mps = 0.0f;
		chassisControl.ChassisRealNeedInput.speed_w_rps = 0.0f;
	}
	/*底盘目标速度映射到轮电机*/
	chassisControl.WheelMotorControl.target_speed_mps[LF] = + chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;

	chassisControl.WheelMotorControl.target_speed_mps[RF] = - chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;

	chassisControl.WheelMotorControl.target_speed_mps[LB] = + chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															- chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;

	chassisControl.WheelMotorControl.target_speed_mps[RB] = - chassisControl.ChassisRealNeedInput.speed_y_mps * 1.414f \
															- chassisControl.ChassisRealNeedInput.speed_x_mps * 1.414f \
															+ chassisControl.ChassisRealNeedInput.speed_w_rps * ROBOT_RPS_TO_WHEEL_MPS;


	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		chassisControl.WheelMotorControl.target_speed_mps[i] *= -1;
	}



}

static void chassis_update_max_compensate_power(void)
{
#ifdef OLD_CAPACITY
    if(_superCapacity->cap_volt >= 18)
        chassisControl.SuperCapacity.max_compensate_power = 230;
    else
        chassisControl.SuperCapacity.max_compensate_power = -0.003 * pow(_superCapacity->cap_volt, 3)
                                                                  + 0.35 * pow(_superCapacity->cap_volt, 2) + 80;
#else
    if(_superCapacity->cap_volt >= 15)
        chassisControl.SuperCapacity.max_compensate_power = 200;
    else if(_superCapacity->cap_volt > 10 && _superCapacity->cap_volt < 15)
        chassisControl.SuperCapacity.max_compensate_power = 40 * (_superCapacity->cap_volt - 10);
    else
        chassisControl.SuperCapacity.max_compensate_power = 0;
#endif
}

static void chassis_update_power_counters(uint16_t* speed_count, uint16_t* revolve_exit_count)
{
    static uint16_t speed_count_cycles = 0;
    static uint16_t revolve_exit_count_cycles = 0;
    static uint16_t last_chassis_mode = 0;

    speed_count_cycles++;
    if(!_normRemoteCmd->PCKeyBoard.level_key_A && !_normRemoteCmd->PCKeyBoard.level_key_D
       && !_normRemoteCmd->PCKeyBoard.level_key_W && !_normRemoteCmd->PCKeyBoard.level_key_S
       && pDecisionAO->chassis_mode != CHASSIS_REVOLVE)
        speed_count_cycles = 0;

    if(pDecisionAO->chassis_mode != CHASSIS_REVOLVE && last_chassis_mode == CHASSIS_REVOLVE)
        revolve_exit_count_cycles = 200;
    if(revolve_exit_count_cycles > 0)
        revolve_exit_count_cycles--;
    last_chassis_mode = pDecisionAO->chassis_mode;

    *speed_count = speed_count_cycles;
    *revolve_exit_count = revolve_exit_count_cycles;
}

static float chassis_calculate_permitted_power(uint16_t speed_count, uint16_t revolve_exit_count)
{
    float permitted_power = ext_game_robot_status.chassis_power_limit;

    if(_superCapacity->cap_volt >= 10.4)
    {
        if(_normRemoteCmd->PCKeyBoard.level_key_SHIFT)
            permitted_power += -0.003 * pow(_superCapacity->cap_volt, 3) + 0.35 * pow(_superCapacity->cap_volt, 2);
        else if(speed_count < 200)
            permitted_power *= 1.5;
        else if(_superCapacity->cap_volt >= 17.4)
        {
            if(permitted_power <= 60)
                permitted_power *= 1.95;
            else if(permitted_power == 70)
                permitted_power *= 1.75;
            else if(permitted_power <= 90)
                permitted_power *= 1.65;
            else
                permitted_power *= 1.55;
        }
        else
            permitted_power *= 1.25;

        if(revolve_exit_count)
            permitted_power *= 0.7;
    }
    else if(revolve_exit_count)
    {
        permitted_power *= 0.6;
    }

    if(pDecisionAO->joint_mode == ROBOT_JOINT_MODE_CLIMB)
    {
        if(_robotState->capacity_mode != NO_CAPACITY && _superCapacity->cap_volt >= 10.4)
            permitted_power *= 2.2f;
        else
            permitted_power *= 2.2f;
    }
    else if(pDecisionAO->stand_mode == ROBOT_STAND_MODE_PRE_STAIR
            || pDecisionAO->stand_mode == ROBOT_STAND_MODE_STAIR_UP)
    {
        if(_robotState->capacity_mode != NO_CAPACITY && _superCapacity->cap_volt >= 10.4)
            permitted_power = 50.0 * 2.1f;
        else
            permitted_power = 50.0f * 1.15f;
    }
    else if(_robotState->capacity_mode == NO_CAPACITY)
    {
        permitted_power = ext_game_robot_status.chassis_power_limit;
    }

    return DoubleEdgeLimiter(permitted_power, 0,
                             ext_game_robot_status.chassis_power_limit
                             + chassisControl.SuperCapacity.max_compensate_power);
}

static void chassis_update_power_limit(void)
{
    uint16_t speed_count;
    uint16_t revolve_exit_count;
    float prior_power = chassis_calculate_prior_power();

    chassisControl.ChassisRealNeedInput.power_limit_scale = 1.0;
    chassis_update_max_compensate_power();
    chassis_update_power_counters(&speed_count, &revolve_exit_count);

    float permitted_power = chassis_calculate_permitted_power(speed_count, revolve_exit_count);
    if(prior_power > permitted_power)
        chassisControl.ChassisRealNeedInput.power_limit_scale = permitted_power / prior_power;
}

static void chassis_update_wheel_outputs(void)
{
	//前馈+PD控制底盘电机闭环
	static float wheel_speed_lpf[CHASSIS_MOTOR_NUM] = {0};
	const float wheel_speed_lpf_alpha = 0.3f;  // 一阶低通系数, 截止频率 ≈ 50Hz @ 1kHz (可调 0.15~0.5)

	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		/* 速度反馈低通滤波: 抑制CAN回传量化噪声, 防止D项放大高频抖动 */
		wheel_speed_lpf[i] = wheel_speed_lpf_alpha * _chassisMotorRec[i].mechanical_speed_rpm
		                   + (1.0f - wheel_speed_lpf_alpha) * wheel_speed_lpf[i];

		chassisControl.WheelMotorControl.target_motor_output[i] = PIDUpdate(&(chassisControl.WheelMotorControl.speed_control_pid[i]),\
																			 (chassisControl.WheelMotorControl.target_speed_mps[i] * WHEEL_MPS_TO_WHEEL_RPM\
																			 - wheel_speed_lpf[i]));
		chassisControl.WheelMotorControl.target_motor_output[i] = AbsLimiter(chassisControl.WheelMotorControl.target_motor_output[i] + \
																			chassisControl.WheelMotorControl.target_speed_mps[i] * CHASSIS_MOTOR_FRONTFEED_RATIO, 16000
		)
																					*chassisControl.ChassisRealNeedInput.power_limit_scale;		//功率控制，可能要去掉;去牛魔 原来是16000注意
		//各种情况的最终手段：如果缓存能量非常少了暴力限制
		if(ext_power_heat_data.buffer_energy<10)
			chassisControl.WheelMotorControl.target_motor_output[i]*=0.5;


	}
}

static void chassis_apply_special_output_allocation(void)
{
	if (pDecisionAO->joint_mode == ROBOT_JOINT_MODE_OUTCLIMB )
	{
		float total_output = abs(chassisControl.WheelMotorControl.target_motor_output[LF]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[RF]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[LB]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[RB]);
		float rear_output = abs(chassisControl.WheelMotorControl.target_motor_output[LB]) +
					  abs(chassisControl.WheelMotorControl.target_motor_output[RB]);
		float scale = (rear_output > 1e-3f) ? (total_output / rear_output) : 0.0f;

		chassisControl.WheelMotorControl.target_motor_output[LF] = 0;
		chassisControl.WheelMotorControl.target_motor_output[RF] = 0;
		chassisControl.WheelMotorControl.target_motor_output[LB] = AbsLimiter(
			chassisControl.WheelMotorControl.target_motor_output[LB] * scale* 0.8, 16000);
		chassisControl.WheelMotorControl.target_motor_output[RB] = AbsLimiter(
			chassisControl.WheelMotorControl.target_motor_output[RB] * scale*0.8, 16000);
	}
	/* 上坡模式(CLIMB)：前腿功率1:3分配到后腿 */
	if (pDecisionAO->joint_mode == ROBOT_JOINT_MODE_CLIMB)
	{
		float front_sum = (float)abs(chassisControl.WheelMotorControl.target_motor_output[LF]) +
		                  (float)abs(chassisControl.WheelMotorControl.target_motor_output[RF]);
		float rear_sum  = (float)abs(chassisControl.WheelMotorControl.target_motor_output[LB]) +
		                  (float)abs(chassisControl.WheelMotorControl.target_motor_output[RB]);
		float total = front_sum + rear_sum;

		if (total > 1e-3f)
		{
			float front_scale = (front_sum > 1e-3f) ? (total * 0.25f / front_sum) : 0.0f;
			float rear_scale  = (rear_sum  > 1e-3f) ? (total * 0.75f / rear_sum)  : 0.0f;

			chassisControl.WheelMotorControl.target_motor_output[LF] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[LF] * front_scale, 16000);
			chassisControl.WheelMotorControl.target_motor_output[RF] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[RF] * front_scale, 16000);
			chassisControl.WheelMotorControl.target_motor_output[LB] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[LB] * rear_scale, 16000);
			chassisControl.WheelMotorControl.target_motor_output[RB] = (int16_t)AbsLimiter(
				(float)chassisControl.WheelMotorControl.target_motor_output[RB] * rear_scale, 16000);
		}
	}
}

static void chassis_apply_slope_transition(void)
{
	static uint16_t slope_count = 0;
	static uint8_t last_slope = 0, cur_slope = 0;
	if(slope_count)
	{
		slope_count--;
		chassisControl.WheelMotorControl.target_motor_output[LF] = chassisControl.WheelMotorControl.target_motor_output[RF] = 0;
	}
	cur_slope = _robotState->auto_slope;
	if(CONTROL_FROM_REMOTE == pDecisionAO->ctrl_terminal && cur_slope == 0 && cur_slope != last_slope)
		slope_count = 200;
	last_slope = cur_slope;

}

static void chassis_apply_output_guards(void)
{
	#if defined CHASSIS_OFF
		for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
			chassisControl.WheelMotorControl.target_motor_output[i] = 0;
	#endif

	if(CONTROL_STOP == pDecisionAO->ctrl_terminal)// || ext_game_robot_status.power_management_chassis_output == 0)
	{
		for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
			chassisControl.WheelMotorControl.target_motor_output[i] = 0;
	}
}

void ChassisControlUpdate(void)
{
    chassis_prepare_control_target();
    chassis_update_power_limit();
    chassis_update_wheel_outputs();
    chassis_apply_special_output_allocation();
    chassis_apply_slope_transition();
    chassis_apply_output_guards();
}

float prior_chassis_power = 0;//底盘功率模型先验计算值
static float chassis_calculate_prior_power(void)
{
	prior_chassis_power = 0;//底盘功率模型先验计算值
	int16_t tatget_motor_output = 0;
	float prior_motor_power = 0;

	const float toque_coefficient = 1.99688994e-6f;//功率模型的第一项系数，物理规律，固定
	const float k_1 = 4e-07;//k_1,k_2均为辨识参数，对于同一电机型号，忽略铜损等电机差异性损耗，都采用同一参数
	const float k_2 = 2.5e-07;
	const float const_cost = 0.5;//除电机外的其他损耗，如电调功率


	for(uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		tatget_motor_output = PIDUpdatePrior(chassisControl.WheelMotorControl.speed_control_pid[i],\
											(chassisControl.WheelMotorControl.target_speed_mps[i] * WHEEL_MPS_TO_WHEEL_RPM\
											- _chassisMotorRec[i].mechanical_speed_rpm))\
							+ chassisControl.WheelMotorControl.target_speed_mps[i] * CHASSIS_MOTOR_FRONTFEED_RATIO;

		tatget_motor_output = AbsLimiter(tatget_motor_output, 16000);

		prior_motor_power = tatget_motor_output * _chassisMotorRec[i].mechanical_speed_rpm * toque_coefficient +\
							k_1 * square(_chassisMotorRec[i].mechanical_speed_rpm) +\
							k_2 * square(tatget_motor_output) +\
							const_cost;
		prior_chassis_power += prior_motor_power;
	}
	prior_chassis_power = AverageFilterUpdate(&PowerFilter, prior_chassis_power);
	return prior_chassis_power;
}

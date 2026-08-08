/**
 * @file gimbalControl.c
 * @brief 云台控制实现：输入决策 + 姿态观测 + ADRC/LTD闭环 + 狙击/关节模式
 * @note  从 robot_control_task.c 拆分，参照 hero_down MainControl 架构
 */

#include "gimbalControl.h"
// 使用单独 include 避免与 robot_control_task.h 的重复 typedef 冲突
// 后续 general_task_include.h 更新后可改为 #include "general_task_include.h"
#include <stdint.h>
#include <math.h>
#include "algorism.h"
#include "pid.h"
#include "adrc.h"
#include "state_task.h"
#include "general_config_label.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"
#include "worldGimbal.h"
#include "../../PrivateDrivers/Board2Borad/Board2Board.h"

/* 算法切换: 注释此行切换为 LADRC, 取消注释切换为 LTD+双环PID */
#define YAW_DUAL_PID

/* === 全局实例 === */
GimbalControl gimbalControl = {0};
const GimbalControl* _gimbalControl = &gimbalControl;

SmoothFilter MouseFilterX = {0};
SmoothFilter MouseFilterY = {0};

float pitch_angle_from_match = 0;

float kpfric = 17.5;      
float kdfric = 12.5;

/* === 外部引用 === */
extern DJIGMotorRec pitchMotorRec;
extern const DJIGMotorRec* _pitchMotorRec;
extern uint8_t angle_error_flag;
extern uint8_t distance_error_flag;
extern Pose gimbalPose;
extern int64_t circle_angle;
extern const RobotState* _robotState;
extern WorldGimbal worldGimbal;

/* 下板计算完成的云台绝对目标，通过 B2B 0x221 下发。 */
extern volatile float g_b2b_yaw_cmd_d;
extern volatile float g_b2b_pitch_cmd_d;
extern volatile uint8_t g_b2b_yaw_cmd_valid;

/* GimbalPoseUpdate 相关 */
uint8_t shit_delay_count = 0;
float shit_temp_pitch_comp = 0;

/* GimbalControlUpdate 相关 */
int fl_u;
float angle_control;

/* ---- yaw 控制 (从下板搬迁) ---- */
static SmoothFilter yawEncFilter = {0};
static ScalarKalmanFilter yawEncKalmanFilter = {0};

#define YAW_OBSERVATION_FILTER_DT_S             (0.002f)
#define YAW_OBSERVATION_FILTER_ENCODER_ALPHA   (0.9937365f)
#define YAW_OBSERVATION_FILTER_CORRECTION_GAIN (3.1415926f)

typedef struct
{
    float angle_d;
    float encoder_low_d;
    uint8_t initialized;
}YawObservationFilter;

static YawObservationFilter yaw_observation_filter = {0};

static void yaw_observation_filter_reset(YawObservationFilter* filter)
{
    filter->angle_d = 0.0f;
    filter->encoder_low_d = 0.0f;
    filter->initialized = 0U;
}

/**
 * @brief   更新独立 yaw 观测器
 * @param   filter 观测器状态
 * @param   encoder_angle_d 编码器角度，单位 deg
 * @param   terminal_angular_velocity_dps 已去零偏的 IMU 角速度，单位 deg/s
 * @param   dt_s 采样周期，单位 s
 * @retval  融合后的 yaw 观测角度，单位 deg
 * @note    编码器使用约 0.5 Hz 低频校正，避免高频机械不同步进入输出。
 */
static float yaw_observation_filter_update(YawObservationFilter* filter,
                                           float encoder_angle_d,
                                           float terminal_angular_velocity_dps,
                                           float dt_s)
{
    if (!filter->initialized)
    {
        if (isfinite(encoder_angle_d))
        {
            filter->angle_d = AngleLimit(encoder_angle_d, -180.0f, 180.0f);
            filter->encoder_low_d = filter->angle_d;
            filter->initialized = 1U;
        }
        return filter->angle_d;
    }

    if (!isfinite(terminal_angular_velocity_dps))
        terminal_angular_velocity_dps = 0.0f;

    /* 角速度已在 IMU 解算链路中完成零偏扣除，此处只做积分预测。 */
    filter->angle_d = AngleLimit(filter->angle_d + terminal_angular_velocity_dps * dt_s,
                                 -180.0f, 180.0f);

    if (isfinite(encoder_angle_d))
    {
        /* 低通前使用最短角度差，避免在 +/-180 deg 处产生跳变。 */
        float encoder_error_d = AngleLimit(encoder_angle_d - filter->encoder_low_d,
                                           -180.0f, 180.0f);
        filter->encoder_low_d = AngleLimit(filter->encoder_low_d
                                           + (1.0f - YAW_OBSERVATION_FILTER_ENCODER_ALPHA)
                                           * encoder_error_d,
                                           -180.0f, 180.0f);

        /* 低频比例校正只限制积分漂移，不把编码器高频抖动带入输出。 */
        float correction_error_d = AngleLimit(filter->encoder_low_d - filter->angle_d,
                                              -180.0f, 180.0f);
        filter->angle_d = AngleLimit(filter->angle_d
                                     + YAW_OBSERVATION_FILTER_CORRECTION_GAIN
                                     * correction_error_d * dt_s,
                                     -180.0f, 180.0f);
    }

    return filter->angle_d;
}

float GimbalControlGetYawObservationAngleD(void)
{
    return yaw_observation_filter.angle_d;
}

float yaw_dm_forward_offset_rad = -2.54f;  /* DM yaw编码器"云台正前方"机械角(rad) */

/* ==================== 函数实现 ==================== */

void GimbalInputUpdate(void)//task1,更新机械限位角度
{
    const float pitch_upside_limit_offset = 41.5;
    const float pitch_downside_limit_offset = -7;

    switch(_robotState->ctrl_terminal)
    {
        case CONTROL_STOP:
            gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
            gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalEstimate.pitch_angle_d;
            gimbalControl.GimbalTargetInput.yaw_angular_velocity_dps = 0;
        break;
        case CONTROL_FROM_REMOTE:
        case CONTROL_FROM_PC:
            /* 下板是唯一目标生产者，上板只消费 0x221 的最终 yaw/pitch。 */
            if(g_b2b_yaw_cmd_valid && g_b2b_down_valid)
            {
                gimbalControl.GimbalTargetInput.yaw_angle_d = g_b2b_yaw_cmd_d;
                gimbalControl.GimbalTargetInput.pitch_angle_d = g_b2b_pitch_cmd_d;
            }
            else
            {
                gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
                gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalEstimate.pitch_angle_d;
            }
        break;
    }

    /*角度限幅*/

    gimbalControl.GimbalTargetInput.yaw_angle_d = AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d, -180, 180);

    if(_robotState->chassis_mode == CHASSIS_REVOLVE)
    {
        gimbalControl.GimbalTargetInput.pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.pitch_angle_d,
                                                                         pitch_downside_limit_offset,
                                                                         pitch_upside_limit_offset);
    }
    else
    {
        gimbalControl.GimbalTargetInput.pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.pitch_angle_d,
                                                                         pitch_downside_limit_offset,
                                                                         pitch_upside_limit_offset);
    }
}

void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w,float roll_angle,float roll_angle_w)
{
    gimbalControl.GimbalEstimate.roll_angle_d=roll_angle;
    gimbalControl.GimbalEstimate.roll_angular_velocity_dps=roll_angle_w;
    pitch_angle_from_match= (_pitchMotorRec->mechanical_angle - PITCH_OFFSET_MACHENICAL_ANGLE) * 360.0f /LK_FULL_CIRCLE_MECHENICAL_ANGLE;
    pitch_angle_from_match= AngleLimit(pitch_angle_from_match, -180, 180);

    /* yaw 角度：SNIPER_ON 走 DM 编码器 + 标量卡尔曼滤波 */
    extern DMJ4310MotorRec DMyawMotorRec;
    float yaw_enc_rad = DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad;
    float yaw_enc_observation_d = (DMyawMotorRec.frame_counter > 0U && isfinite(yaw_enc_rad))
                                      ? AngleLimit(yaw_enc_rad * 57.29578f, -180.0f, 180.0f)
                                      : NAN;
    yaw_observation_filter_update(&yaw_observation_filter,
                                  yaw_enc_observation_d,
                                  yaw_angle_w * 57.29578f,
                                  YAW_OBSERVATION_FILTER_DT_S);
    static uint8_t yaw_kf_realign = 1U;
    if(_robotState->sniper == SNIPER_ON)
    {
        float yaw_vel_dps  = -DMyawMotorRec.vel_radps * 57.29578f;
        gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = yaw_angle_w * 57.3f;
        if(isfinite(yaw_enc_rad))
        {
            float z = AngleLimit(yaw_enc_rad * 57.29578f, -180.0f, 180.0f);
            float u = isfinite(yaw_vel_dps) ? yaw_vel_dps : 0.0f;
            if(yaw_kf_realign)
            {
                ScalarKalmanFilterReset(&yawEncKalmanFilter, z);
                yaw_kf_realign = 0U;
            }
            gimbalControl.GimbalEstimate.yaw_angle_d = ScalarKalmanFilterUpdate(&yawEncKalmanFilter, z, u);
        }
    }
    else
    {
        yaw_kf_realign = 1U;
        gimbalControl.GimbalEstimate.yaw_angle_d = yaw_angle;
        gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = yaw_angle_w * 57.3f;
    }

    /* sniper过渡检测：切换时对齐目标 + 复位 ADRC 状态 */
    {
        static uint8_t prev_sniper = SNIPER_OFF;
        if(prev_sniper != _robotState->sniper)
        {
            shit_delay_count = 0;
            gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
#ifdef YAW_DUAL_PID
            LTD_Reset(&gimbalControl.GimbalMotorControl.yaw_LTD, gimbalControl.GimbalEstimate.yaw_angle_d);
            PIDReset(&gimbalControl.GimbalMotorControl.yaw_speed_pid);
#else
            {
                float yaw_reset_rad = gimbalControl.GimbalEstimate.yaw_angle_d * (3.141592f / 180.0f);
                TD_Reset(&gimbalControl.GimbalMotorControl.yaw_ADRC.td, yaw_reset_rad);
                ESO_Reset(&gimbalControl.GimbalMotorControl.yaw_ADRC.eso, yaw_reset_rad);
            }
#endif
            prev_sniper = _robotState->sniper;
        }
    }
    if(shit_delay_count < 200)
        shit_delay_count++;
    gimbalControl.GimbalEstimate.pitch_angular_velocity_dps = pitch_angle_w * 57.3f;
        //最后手段,机械角
        if(_robotState->sniper==SNIPER_OFF )
            gimbalControl.GimbalEstimate.pitch_angle_d = pitch_angle+shit_temp_pitch_comp;
        if(_robotState->sniper==SNIPER_ON )
            gimbalControl.GimbalEstimate.pitch_angle_d=pitch_angle_from_match;

    static uint8_t last_state = 0, cur_state;

}

void GimbalInit(void)
{
    /* 世界系云台控制初始化 */
    WorldGimbalInit(&worldGimbal);

    /* ---- pitch 控制初始化 ---- */
    gimbalControl.GimbalTargetInput.pitch_angle_d = 4.9f;
    float h_temp = CONTROL_TASK_PERIOD_SET / 1000.0;
    //狗屎
    LTDInitialize(&gimbalControl.GimbalMotorControl.pitch_LTD, 20, 0.003, -30, 60);
    gimbalControl.GimbalMotorControl.pitch_LTD.ki1=0.02;
    gimbalControl.GimbalMotorControl.pitch_LTD.error_sum=0;
    gimbalControl.GimbalMotorControl.pitch_LTD.error_sum_max=90;
    gimbalControl.GimbalMotorControl.pitch_LTD.lv_bo=0.3;

    /* pitch ADRC: w0=20, b0=3 */
    {
        const float eso_wo = 20.0f;
        float td_init[3]   = {0.0f, h_temp, 0.0f};
        float lesf_init[5] = {0.0f, 70.0f, 8.0f, 50.0f, 1000.0f};                 /* k0,k1,k2,e0_max,out_limit */
        float eso_init[6]  = {h_temp, 3.0f, 3.0f*eso_wo, 3.0f*eso_wo*eso_wo,
                               0.30f*eso_wo*eso_wo*eso_wo, 2000.0f};                /* h,b,β01,β02,β03,z3_limit */
        LADRCInitialize(&gimbalControl.GimbalMotorControl.pitch_angle_adrc, td_init, lesf_init, eso_init, 0.0f, 0.0f);
    }
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1_min = -15;
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1_max = +36;
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2_min = -200;
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2_max = +200;
    /* ---- yaw 控制初始化 (从下板搬迁) ---- */
    int yaw_wc = 6, yaw_w0 = 22; //w0 = 3~10*wc
    /* yaw LADRC */
    float td_init[3]   = {20.0f, 0.002f, 1.0f};                                      // r, h0, N
    float lesf_init[5] = {0.0f, (float)(yaw_wc*yaw_wc), (float)(2*yaw_wc), 0.0f, 50000.0f}; // k_0, k_1, k_2, e_0_max, output_limit
    float eso_init[6]  = {0.002f, 40, (float)(3*yaw_w0), (float)(3*yaw_w0*yaw_w0), (float)(yaw_w0*yaw_w0*yaw_w0), 10000.0f}; // h, b, β01, β02, β03, z3_limit
    LADRCInitialize(&gimbalControl.GimbalMotorControl.yaw_ADRC, td_init, lesf_init, eso_init, -3.14159f, 3.14159f);
    SmoothFilterInitialize(&yawEncFilter, 0.7f);
    ScalarKalmanFilterInit(&yawEncKalmanFilter, 0.0000001f, 0.0000188825f, 0.002f);
    yaw_observation_filter_reset(&yaw_observation_filter);

#ifdef YAW_DUAL_PID
    LTDInitialize(&gimbalControl.GimbalMotorControl.yaw_LTD, 20, 0.002, -180, 180);
    PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_pos_pid,   7.0, 0.006, 0.00,    1000, 400);
    PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_speed_pid, 0.13, 0.0, 0.0, 0, 5);
#endif
}

void GimbalControlUpdate(void)
{
    static uint8_t last_joint_mode = 0;
    static uint8_t joint_delay_count = 30;
    uint8_t joint_mode = (_robotState->joint_mode == ROBOT_JOINT_MODE_CLIMB);

    if(joint_mode != last_joint_mode)
    {//超级手工答辩
        joint_delay_count = 0;
        gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalEstimate.pitch_angle_d;
        gimbalControl.GimbalMotorControl.pitch_LTD.error_sum = 0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1 = 0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2 = 0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z3 = 0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.esf.output = 0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.u_0 = 0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.u = 0;
        gimbalControl.GimbalMotorControl.pitch_target_output = 0;
    }
    if(joint_delay_count < 30)
        joint_delay_count++;
    last_joint_mode = joint_mode;
    /*不同状态参数不一样
    是否有吊射状态*/
        float output_limit=800;//gimbalControl.GimbalEstimate.pitch_angular_velocity_dps
        LTDADRCUpdate(&gimbalControl.GimbalMotorControl.pitch_angle_adrc,&gimbalControl.GimbalMotorControl.pitch_LTD, gimbalControl.GimbalTargetInput.pitch_angle_d,gimbalControl.GimbalEstimate.pitch_angle_d);
        uint32_t zero_pos=244360;//测一下
        fl_u=gimbalControl.GimbalMotorControl.pitch_angle_adrc.u*1+fl_u*0;
        if(_robotState->sniper==SNIPER_ON)//右下吊射模式用编码器精度为0.01°
        {
        angle_control=(gimbalControl.GimbalTargetInput.pitch_angle_d*1000);
        angle_control=DoubleEdgeLimiter(angle_control,-9000,30000);//限位-10-33

        gimbalControl.GimbalMotorControl.sniper_pos=zero_pos+angle_control;
       // gimbalControl.GimbalMotorControl.sniper_pos =264460;
        gimbalControl.GimbalMotorControl.spin_dir=(gimbalControl.GimbalMotorControl.sniper_pos>circle_angle)?0x00:0x01;//0x00顺时针,0x01逆时针
        gimbalControl.GimbalMotorControl.sniper_max_speed=50;//转动最大速度,单位dps
            //看看编码器向上转是正还是负
            //控制逻辑,测量零度的时候编码器对应值,要经过0度时编码器值(0.01+或-angle_control,看看imu在0度对应24436
            //spin_dir看Input与estimate之差,符号函数//task01,逻辑完善
        }
        else
        {
        gimbalControl.GimbalMotorControl.pitch_target_output=AbsLimiter(fl_u,output_limit);
        }
      if(_robotState->joint_mode==ROBOT_JOINT_MODE_CLIMB)
        {
        angle_control=(gimbalControl.GimbalTargetInput.pitch_angle_d*1000);
        angle_control=DoubleEdgeLimiter(angle_control,-9000,30000);//限位-10-33

        gimbalControl.GimbalMotorControl.sniper_pos=zero_pos+angle_control;
        gimbalControl.GimbalMotorControl.spin_dir=(gimbalControl.GimbalMotorControl.sniper_pos>circle_angle)?0x00:0x01;//0x00顺时针,0x01逆时针
        gimbalControl.GimbalMotorControl.sniper_max_speed=50;//转动最大速度,单位dps
        }

    /* ===== yaw 轴控制 (从下板搬迁) ===== */
#ifdef YAW_DUAL_PID
		/* ===== LTD + 双环PID 控制（原始版本：位置环输出作速度给定 + LTD前馈） ===== */
		/* SNIPER_ON 使用终端观测器；SNIPER_OFF 保持 IMU yaw 坐标系。 */
		float yaw_feedback_d = gimbalControl.GimbalEstimate.yaw_angle_d;
		if (_robotState->sniper == SNIPER_ON && yaw_observation_filter.initialized)
		{
			yaw_feedback_d = GimbalControlGetYawObservationAngleD();
		}
		float yaw_pos_err_d = AngleLimit(gimbalControl.GimbalMotorControl.yaw_LTD.x1 - yaw_feedback_d,
		                                 -180.0f, 180.0f);
		float pre_yaw_Tff = 0;
		// if(fabs(yaw_pos_err_d) < 0.5f)
		// {
		// 	/* 小误差：强位置保持，不加前馈 */
		// 	gimbalControl.GimbalMotorControl.yaw_pos_pid.ki = 0.1f;
		// 	gimbalControl.GimbalMotorControl.yaw_pos_pid.kp = 8.0f;
		// 	pre_yaw_Tff = 0.0f;
		// }
		// else
		// {
		// 	/* 大误差：清积分防超调 + LTD速度前馈加速追赶 */
		// 	gimbalControl.GimbalMotorControl.yaw_pos_pid.ki = 0.0f;
		// 	gimbalControl.GimbalMotorControl.yaw_pos_pid.sum_error = 0.0f;
		// 	pre_yaw_Tff = gimbalControl.GimbalMotorControl.yaw_LTD.x2 * 0.02f;
		// }
		LTDUpdate(&gimbalControl.GimbalMotorControl.yaw_LTD, gimbalControl.GimbalTargetInput.yaw_angle_d);
		PIDUpdate(&gimbalControl.GimbalMotorControl.yaw_pos_pid, yaw_pos_err_d);
		PIDUpdateWithFilteredD(&gimbalControl.GimbalMotorControl.yaw_speed_pid,
			  gimbalControl.GimbalMotorControl.yaw_pos_pid.output - gimbalControl.GimbalEstimate.yaw_angular_velocity_dps,
			  0.5f);
		gimbalControl.GimbalMotorControl.yaw_target_output = -(gimbalControl.GimbalMotorControl.yaw_speed_pid.output + pre_yaw_Tff);
		gimbalControl.GimbalMotorControl.w_d = gimbalControl.GimbalMotorControl.yaw_pos_pid.output;
		gimbalControl.GimbalTargetInput.yaw_angular_velocity_dps = gimbalControl.GimbalMotorControl.yaw_pos_pid.output;
#else
		/* ===== LADRC 角度环控制 ===== */
		YawLADRCUpdate(&gimbalControl.GimbalMotorControl.yaw_ADRC,
		            gimbalControl.GimbalTargetInput.yaw_angle_d * (3.141592f / 180.0f),
		            gimbalControl.GimbalEstimate.yaw_angle_d * (3.141592f / 180.0f));
		gimbalControl.GimbalMotorControl.yaw_target_output = -(gimbalControl.GimbalMotorControl.yaw_ADRC.u);
#endif

    /*共同保护*/
    if(CONTROL_STOP == _robotState->ctrl_terminal || shit_delay_count < 30 || joint_delay_count < 30)
    {
        gimbalControl.GimbalTargetInput.yaw_angle_d=gimbalControl.GimbalEstimate.yaw_angle_d;

        gimbalControl.GimbalMotorControl.pitch_LTD.error_sum=0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1=0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2=0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z3=0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.esf.output=0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.u_0=0;
        gimbalControl.GimbalMotorControl.pitch_angle_adrc.u=0;

        gimbalControl.GimbalMotorControl.pitch_target_output = 0;


	        /* yaw 保护复位 */
#ifdef YAW_DUAL_PID
	        LTD_Reset(&gimbalControl.GimbalMotorControl.yaw_LTD, gimbalControl.GimbalEstimate.yaw_angle_d);
	        PIDReset(&gimbalControl.GimbalMotorControl.yaw_speed_pid);
	        gimbalControl.GimbalMotorControl.yaw_LTD.error_sum = 0;
#else
	        {
	            float yaw_reset_rad = gimbalControl.GimbalEstimate.yaw_angle_d * (3.141592f / 180.0f);
	            TD_Reset(&gimbalControl.GimbalMotorControl.yaw_ADRC.td, yaw_reset_rad);
	            ESO_Reset(&gimbalControl.GimbalMotorControl.yaw_ADRC.eso, yaw_reset_rad);
	        }
#endif
	        gimbalControl.GimbalMotorControl.yaw_target_output = 0;
    }

    #if defined GIMBAL_OFF
        gimbalControl.GimbalMotorControl.pitch_target_output = 0;
    #endif
}

void GimbalEstimateUpdate(void)
{
    /* 保持空函数体，作为EstimateTask调用占位 */
}

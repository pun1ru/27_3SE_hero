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
extern UpperComputerComm upperComputerComm;
extern const UpperComputerComm* _upperComputerComm;
extern RobotState robotState;
extern const RobotState* _robotState;
extern const NormRemoteCmd* _normRemoteCmd;
extern WorldGimbal worldGimbal;

/* 世界系辅助函数（定义在 robot_control_task.c） */
extern void WG_WorldAnglesToFdesB(float world_yaw_deg, float world_pitch_deg,
                                   const float* g_B, float* f_des_B_out);

/* === 模块级变量 === */
/* GimbalInputUpdate 相关 */
float micro_pitch = 0;
int temp_pitch_count = 0, last_temp_pitch = 0;

/* GimbalPoseUpdate 相关 */
uint8_t shit_delay_count = 0;
float shit_temp_pitch_comp = 0;

/* GimbalControlUpdate 相关 */
int fl_u;
float angle_control;

/* ---- yaw 控制 (从下板搬迁) ---- */
static SmoothFilter yawEncFilter = {0};
static ScalarKalmanFilter yawEncKalmanFilter = {0};
static uint8_t c_key_aim_yaw_lock = 0;
float yaw_dm_forward_offset_rad = -2.54f;  /* DM yaw编码器"云台正前方"机械角(rad) */
uint8_t yaw_shit_delay_count = 0;

/* ==================== 函数实现 ==================== */

void GimbalInputUpdate(void)//task1,更新机械限位角度
{
    const float pitch_upside_limit_offset = 43;
    const float pitch_downside_limit_offset = -7;
    static uint16_t count = 0;
    float smooth=0.02,smoothtry=0.015;
    float micro_change=1;
    switch(_robotState->ctrl_terminal)
    {
        case CONTROL_STOP:
            gimbalControl.GimbalTargetInput.yaw_angle_d = gimbalControl.GimbalEstimate.yaw_angle_d;
            gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalEstimate.pitch_angle_d;
            gimbalControl.GimbalTargetInput.yaw_angular_velocity_dps = 0;
        break;
        case CONTROL_FROM_REMOTE:
            if((_robotState->sniper==SNIPER_ON)){
                if(_normRemoteCmd->RelativeCH.ch0>0.1)//算法测试用
                {
                gimbalControl.GimbalTargetInput.yaw_angle_d=upperComputerComm.Receive.target_yaw_angle_d;
                gimbalControl.GimbalTargetInput.pitch_angle_d=upperComputerComm.Receive.target_pitch_angle_d;
                }
                else
                {
                /* CH2→dyaw, CH3→dpitch: 世界系模式走虚拟目标，否则直接改电机角 */
                float dyaw = _normRemoteCmd->RelativeCH.ch2 * 0.1f - _normRemoteCmd->RelativeCH.ch2 * (_robotState->chassis_mode == CHASSIS_SEPARATE);
                float dpitch = 0.03f * (_normRemoteCmd->RelativeCH.ch3 * 2.0f - _normRemoteCmd->RelativeCH.ch3 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
                if (worldGimbal.enable) {
                    WorldGimbalInputUpdate(&worldGimbal, dyaw, dpitch);
                } else {
                    gimbalControl.GimbalTargetInput.yaw_angle_d += dyaw;
                    gimbalControl.GimbalTargetInput.pitch_angle_d += dpitch;
                }
                }
            }
            if(_robotState->sniper==SNIPER_OFF){
                gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 2.5 - _normRemoteCmd->RelativeCH.ch2 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
                gimbalControl.GimbalTargetInput.pitch_angle_d -= (-0.5)*(_normRemoteCmd->RelativeCH.ch3 * 2 - _normRemoteCmd->RelativeCH.ch3 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
            }

        break;
        case CONTROL_FROM_PC:

              if(_robotState->aim_mode && _robotState->sniper == SNIPER_ON)//C键单次追逐上位机目标角度
            {
                if (worldGimbal.enable) {
                    /* 世界系模式：上位机发送世界系yaw/pitch → 转换为机体向量f_des_B → IK反解电机角 */
                    WG_WorldAnglesToFdesB(
                        _upperComputerComm->Receive.target_yaw_angle_d,
                        _upperComputerComm->Receive.target_pitch_angle_d,
                        worldGimbal.WorldGimbalEstimate.g_B,
                        worldGimbal.WorldGimbalTargetInput.f_des_B);
                    worldGimbal.WorldGimbalTargetInput.init_done = 1;
                    worldGimbal.WorldGimbalTargetInput.last_right_valid = 0;
                } else {
                    gimbalControl.GimbalTargetInput.pitch_angle_d = _upperComputerComm->Receive.target_pitch_angle_d;
                    gimbalControl.GimbalTargetInput.yaw_angle_d = _upperComputerComm->Receive.target_yaw_angle_d;
                }
                robotState.aim_mode = 0;  // 单次触发后立即清零，不进入持续追逐
            }
            else
            {
                float temp_pitch = 0;
                float temp_yaw = 0;
                if(_robotState->sniper==SNIPER_OFF){
                    smooth=smooth;
                }
                if((_robotState->sniper==SNIPER_ON)){
                    int temp_pitch_an = (_normRemoteCmd->PCKeyBoard.level_key_W-_normRemoteCmd->PCKeyBoard.level_key_S);
                        /*正在WASD的时候*/
                        if (temp_pitch_an !=0){
                            last_temp_pitch=temp_pitch_an;
                            temp_pitch_count++;
                            if(temp_pitch_count>10){
                                temp_pitch=(_normRemoteCmd->PCKeyBoard.level_key_W-_normRemoteCmd->PCKeyBoard.level_key_S)*0.01;
                                micro_pitch+=(_normRemoteCmd->PCKeyBoard.level_key_W-_normRemoteCmd->PCKeyBoard.level_key_S)*0.01;
                            }
                        }
                    /*狙击WASD结束的时候*/
                    if (temp_pitch_an ==0 && last_temp_pitch!=0){
                        if(temp_pitch_count<=10){//短按动一格子
                            micro_pitch+=last_temp_pitch*0.1;
                            temp_pitch +=last_temp_pitch*0.1;
                    }
                            last_temp_pitch=0;
                            temp_pitch_count=0;
                    }

                    smooth=smoothtry;
                }
                /* mouse_fix ON + sniper ON 时禁用鼠标角度输入，仅保留WASD */
                if(!(_robotState->sniper == SNIPER_ON && _robotState->mouse_fix == MOUSE_FIX_ON)){
                    temp_pitch += SmoothFilterUpdate(&MouseFilterY,_normRemoteCmd->PCMouse.mouse_speed_y)*smooth;
                    temp_yaw += SmoothFilterUpdate(&MouseFilterX,_normRemoteCmd->PCMouse.mouse_speed_x)*smooth;
                }
                micro_pitch=0;
                //最终整定
                /* ---- 世界系模式：指令累积到虚拟目标 f_des_B，不直接改电机角 ---- */
                /* 仅 sniper_on 时允许世界系控制，常规模式强制走普通云台控制 */
                if (worldGimbal.enable && _robotState->sniper == SNIPER_ON) {
                    WorldGimbalInputUpdate(&worldGimbal, temp_yaw, temp_pitch);
                } else {
                    gimbalControl.GimbalTargetInput.pitch_angle_d += temp_pitch;
                if(_robotState->follow!=FOLLOW_ON)
                    gimbalControl.GimbalTargetInput.yaw_angle_d += temp_yaw;
                }
            }

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
    if(_robotState->sniper == SNIPER_ON)
    {
        float yaw_enc_rad = DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad;
        float yaw_vel_dps  = -DMyawMotorRec.vel_radps * 57.29578f;
        gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = yaw_vel_dps;
        if(isfinite(yaw_enc_rad))
        {
            float z = AngleLimit(yaw_enc_rad * 57.29578f, -180.0f, 180.0f);
            float u = isfinite(yaw_vel_dps) ? yaw_vel_dps : 0.0f;
            gimbalControl.GimbalEstimate.yaw_angle_d = ScalarKalmanFilterUpdate(&yawEncKalmanFilter, z, u);
        }
    }
    else
    {
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

#ifdef YAW_DUAL_PID
    LTDInitialize(&gimbalControl.GimbalMotorControl.yaw_LTD, 20, 0.002, -180, 180);
    PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_pos_pid,   8.5, 0.15, 0,    4000, 800);
    PIDInitialize(&gimbalControl.GimbalMotorControl.yaw_speed_pid, 0.09, 0.0, 0.01, 0, 10);
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
        // Align target and reset ADRC states on joint_mode transition.
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
		float yaw_pos_err_d = AngleLimit(gimbalControl.GimbalMotorControl.yaw_LTD.x1 - gimbalControl.GimbalEstimate.yaw_angle_d, -180.0f, 180.0f);
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
		PIDUpdate(&gimbalControl.GimbalMotorControl.yaw_speed_pid,
			  gimbalControl.GimbalMotorControl.yaw_pos_pid.output - gimbalControl.GimbalEstimate.yaw_angular_velocity_dps);
		gimbalControl.GimbalMotorControl.yaw_target_output = -(gimbalControl.GimbalMotorControl.yaw_speed_pid.output + pre_yaw_Tff);
		gimbalControl.GimbalMotorControl.w_d = gimbalControl.GimbalMotorControl.yaw_pos_pid.output;
		gimbalControl.GimbalTargetInput.yaw_angular_velocity_dps = gimbalControl.GimbalMotorControl.yaw_pos_pid.output;
#else
		/* ===== LADRC 角度环控制 ===== */
		LADRCUpdate(&gimbalControl.GimbalMotorControl.yaw_ADRC,
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
    }

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

    #if defined GIMBAL_OFF
        gimbalControl.GimbalMotorControl.pitch_target_output = 0;
    #endif
}

void GimbalEstimateUpdate(void)
{
    /* 保持空函数体，作为EstimateTask调用占位 */
}

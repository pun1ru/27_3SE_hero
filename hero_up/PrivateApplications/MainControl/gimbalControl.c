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

/* === 全局实例 === */
GimbalControl gimbalControl = {0};
const GimbalControl* _gimbalControl = &gimbalControl;

SmoothFilter MouseFilterX = {0};
SmoothFilter MouseFilterY = {0};

float pitch_angle_from_match = 0;

float kpfric = 17.5;       //hxgpid
float kdfric = 12.5;

/* === 外部引用 === */
extern DJIGMotorRec pitchMotorRec;
extern const DJIGMotorRec* _pitchMotorRec;
extern DJIGMotorRec smallpitchMotorRec;
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

/* ==================== 函数实现 ==================== */

void GimbalInputUpdate(void)//task1,更新机械限位角度
{
    /*物理限位为-23~38度，上下限的offset值为达到机械上下限位时，底盘水平状态下imu的反馈pitch角*/
    const float pitch_upside_limit_offset = 43;
    const float pitch_upside_revolve_limit_offset = 43;	                     //xianfu
    const float pitch_downside_limit_offset = -7;
    const float small_pitch_upside_limit_offset = 43;
    const float small_pitch_upside_revolve_limit_offset = 43;
    const float small_pitch_downside_limit_offset = -7;
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
        //改一下逻辑debug
        case CONTROL_FROM_REMOTE:
            if((_robotState->sniper==SNIPER_ON)){//task2,修改遥控器pitch输入,小pitch=大pitch输入,其实没什么改得
                if(_normRemoteCmd->RelativeCH.ch0>0.1)//吊射模式
                {
                gimbalControl.GimbalTargetInput.yaw_angle_d=upperComputerComm.Receive.target_yaw_angle_d;
                gimbalControl.GimbalTargetInput.small_pitch_angle_d=upperComputerComm.Receive.target_pitch_angle_d;
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
                    gimbalControl.GimbalTargetInput.small_pitch_angle_d += dpitch;
                }
                }
            }
            if(_robotState->sniper==SNIPER_OFF){
                gimbalControl.GimbalTargetInput.yaw_angle_d += (_normRemoteCmd->RelativeCH.ch2 * 2.5 - _normRemoteCmd->RelativeCH.ch2 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
                gimbalControl.GimbalTargetInput.small_pitch_angle_d -= (-0.5)*(_normRemoteCmd->RelativeCH.ch3 * 2 - _normRemoteCmd->RelativeCH.ch3 * (_robotState->chassis_mode == CHASSIS_SEPARATE));
            }
            /*计算状态*/
                if(_robotState->follow==FOLLOW_OFF){
                    gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalTargetInput.small_pitch_angle_d;
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
                    gimbalControl.GimbalTargetInput.small_pitch_angle_d = _upperComputerComm->Receive.target_pitch_angle_d;
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
                /*计算状态*/
                if(_robotState->follow==FOLLOW_OFF){
                    gimbalControl.GimbalTargetInput.pitch_angle_d = gimbalControl.GimbalTargetInput.small_pitch_angle_d;
                    /* mouse_fix ON + sniper ON 时禁用鼠标角度输入，仅保留WASD */
                    if(!(_robotState->sniper == SNIPER_ON && _robotState->mouse_fix == MOUSE_FIX_ON)){
                        temp_pitch += SmoothFilterUpdate(&MouseFilterY,_normRemoteCmd->PCMouse.mouse_speed_y)*smooth;
                        temp_yaw += SmoothFilterUpdate(&MouseFilterX,_normRemoteCmd->PCMouse.mouse_speed_x)*smooth;
                    }
                    micro_pitch=0;
                    /*非锁定状态下鼠标PY可动*/
                }
                //最终整定
                /* ---- 世界系模式：指令累积到虚拟目标 f_des_B，不直接改电机角 ---- */
                /* 仅 sniper_on 时允许世界系控制，常规模式强制走普通云台控制 */
                if (worldGimbal.enable && _robotState->sniper == SNIPER_ON) {
                    WorldGimbalInputUpdate(&worldGimbal, temp_yaw, temp_pitch);
                } else {
                    gimbalControl.GimbalTargetInput.small_pitch_angle_d += temp_pitch;
                if(_robotState->follow!=FOLLOW_ON)
                    gimbalControl.GimbalTargetInput.yaw_angle_d += temp_yaw;
                }
            }

    }

    /*角度限幅*/

    gimbalControl.GimbalTargetInput.yaw_angle_d = AngleLimit(gimbalControl.GimbalTargetInput.yaw_angle_d, -180, 180);

    if(_robotState->chassis_mode == CHASSIS_REVOLVE)
    {
            gimbalControl.GimbalTargetInput.pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.pitch_angle_d, \
                                                                             pitch_downside_limit_offset, \
                                                                             pitch_upside_revolve_limit_offset);
        gimbalControl.GimbalTargetInput.small_pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.small_pitch_angle_d, \
                                                                             small_pitch_downside_limit_offset, \
                                                                             small_pitch_upside_revolve_limit_offset);
    }
    else
    {
        gimbalControl.GimbalTargetInput.pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.pitch_angle_d, \
                                                                             pitch_downside_limit_offset, \
                                                                             pitch_upside_limit_offset);//task3,角度限幅
        gimbalControl.GimbalTargetInput.small_pitch_angle_d = DoubleEdgeLimiter(gimbalControl.GimbalTargetInput.small_pitch_angle_d, \
                                                                             small_pitch_downside_limit_offset, \
                                                                             small_pitch_upside_limit_offset);
    }
}

void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w,float roll_angle,float roll_angle_w)
{
    gimbalControl.GimbalEstimate.roll_angle_d=roll_angle;
    gimbalControl.GimbalEstimate.roll_angular_velocity_dps=roll_angle_w;
    pitch_angle_from_match= (_pitchMotorRec->mechanical_angle - PITCH_OFFSET_MACHENICAL_ANGLE) * 360.0f /LK_FULL_CIRCLE_MECHENICAL_ANGLE;//符号注意，记得改这玩意
    pitch_angle_from_match= AngleLimit(pitch_angle_from_match, -180, 180);

    /* hero_up无yaw电机，yaw角度始终用IMU观测 */
    gimbalControl.GimbalEstimate.yaw_angle_d = yaw_angle;

    /* sniper过渡检测：pitch估计切换编码器/IMU时保护控制 */
    {
        static uint8_t prev_sniper = SNIPER_OFF;
        if(prev_sniper != _robotState->sniper)
        {
            shit_delay_count = 0;
            gimbalControl.GimbalTargetInput.yaw_angle_d = yaw_angle;
            prev_sniper = _robotState->sniper;
        }
    }
    if(shit_delay_count < 200)
        shit_delay_count++;
    gimbalControl.GimbalEstimate.yaw_angular_velocity_dps = yaw_angle_w * 57.3f;
    gimbalControl.GimbalEstimate.pitch_angular_velocity_dps = pitch_angle_w * 57.3f;
        //最后手段,机械角
        if(_robotState->sniper==SNIPER_OFF )
            gimbalControl.GimbalEstimate.pitch_angle_d = pitch_angle+shit_temp_pitch_comp;
        //if(_robotState->sniper==SNIPER_ON ||_robotState->aim_mode==OUTPOSE_MODE)
        if(_robotState->sniper==SNIPER_ON )
            gimbalControl.GimbalEstimate.pitch_angle_d=pitch_angle_from_match;//这里有什么问题为什么不能更新AAAA

    static uint8_t last_state = 0, cur_state;

}

void GimbalInit(void)
{
    /* 世界系云台控制初始化 */
    WorldGimbalInit(&worldGimbal);

    //云台电机
    float pitch_angle_offset_d = 4.9;
    gimbalControl.GimbalTargetInput.pitch_angle_d = pitch_angle_offset_d;
    float h_temp = CONTROL_TASK_PERIOD_SET / 1000.0;

//---------------------------------------pitch-ltd-eso-lsef初始化h=0.003s，
    //min,max,LTD 输出角度参考的安全边界
    LTDInitialize(&gimbalControl.GimbalMotorControl.pitch_LTD, 20, 0.003, -30, 60);//pitch,ltd调r
    //r大响应快,但是可能超调,发散,r小反应慢
    const float pitch_eso_wo=20.0f;//增大带宽噪声不好,使z3跳跳
    const float pitch_eso_b0 = 3.0f;
    const float pitch_z3_limit = 2000.0f;//扰动估计限幅
    const float e_min=0;
    const float e_max=0;
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1_min=-15;
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z1_max=+36;
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2_min=-200;
    gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso.z2_max=+200;//不要限幅,会怪怪的

    ESOInitialize(&gimbalControl.GimbalMotorControl.pitch_angle_adrc.eso, h_temp,
        pitch_eso_b0,
        3.0f*pitch_eso_wo,
        3.0f*pitch_eso_wo*pitch_eso_wo,
        0.30f*pitch_eso_wo*pitch_eso_wo*pitch_eso_wo,//如果只降低z3积分可以减少噪声,相当于滤波?
        pitch_z3_limit, e_min, e_max);
    const float pitch_k_0 = 0.0f;           // I
    const float pitch_k_1 = 70.0f;          // P
    const float pitch_k_2 = 8.0f;           // D
    const float pitch_e_0_max = 50.0f;
    const float pitch_output_limit = 1000.0f;// 输出限幅
    LESFInitialize(&gimbalControl.GimbalMotorControl.pitch_angle_adrc.esf, pitch_k_0, pitch_k_1, pitch_k_2, pitch_e_0_max, pitch_output_limit);
    //LESF初始化
    gimbalControl.GimbalMotorControl.pitch_LTD.ki1=0.02;//还有一个ki   0.01
    gimbalControl.GimbalMotorControl.pitch_LTD.error_sum=0;
    gimbalControl.GimbalMotorControl.pitch_LTD.error_sum_max=90;            //80;
    gimbalControl.GimbalMotorControl.pitch_LTD.lv_bo=0.3;
}

void GimbalControlUpdate(void)
{
    static uint8_t last_joint_mode = 0;
    static uint8_t joint_delay_count = 30;
    uint8_t joint_mode = (_robotState->joint_mode == ROBOT_JOINT_MODE_CLIMB);

    if(joint_mode != last_joint_mode)
    {
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
       // gimbalControl.GimbalMotorControl.sniper_pos =264460;
        gimbalControl.GimbalMotorControl.spin_dir=(gimbalControl.GimbalMotorControl.sniper_pos>circle_angle)?0x00:0x01;//0x00顺时针,0x01逆时针
        gimbalControl.GimbalMotorControl.sniper_max_speed=50;//转动最大速度,单位dps
        }

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
        gimbalControl.GimbalMotorControl.small_pitch_target_output = 0;
    }

// 	/*望远瞄具舵机决策+控制+发送*/

    //if(CONTROL_STOP != _robotState->ctrl_terminal)
    //{
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
// 		if(_robotState->lens==LENS_OFF)//调试用
// 			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,duoji);
// 		if(_robotState->lens==LENS_ON)
// 			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,duoji2);



    #if defined GIMBAL_OFF
        gimbalControl.GimbalMotorControl.pitch_target_output = 0;
    #endif
}

void GimbalEstimateUpdate(void)
{
    /* 保持空函数体，作为EstimateTask调用占位 */
}

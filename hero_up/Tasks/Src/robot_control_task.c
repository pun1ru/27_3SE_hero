#include "robot_control_task.h"
#include "state_task.h"
#include "stdio.h"
#include "math.h"
#include "tim.h"
#include "general_task_include.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"
#include <arm_math.h>
#include <stdint.h>
#include "CAN_driver.h"
#include "DMJ4310.h"
#include "shoot_speed_best_contrl.h"
#include "LK_driver.h"
#include "rotation_Martix.h"
#include "peripheral_receive_task.h"
#include "peripheral_transmit_task.h"

/* ========== 世界系云台控制：向量数学工具 ========== */
#define WG_DEG2RAD(x) ((x) * 0.01745329252f)   /* PI/180 */
#define WG_RAD2DEG(x) ((x) * 57.2957795131f)   /* 180/PI */

static inline float wg_dot3(const float* a, const float* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static inline void wg_cross3(const float* a, const float* b, float* r) {
    r[0] = a[1]*b[2] - a[2]*b[1];
    r[1] = a[2]*b[0] - a[0]*b[2];
    r[2] = a[0]*b[1] - a[1]*b[0];
}
static inline float wg_norm3(const float* v) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static inline void wg_normalize3(float* v) {
    float n = wg_norm3(v);
    if (n > 1e-9f) { float inv = 1.0f/n; v[0]*=inv; v[1]*=inv; v[2]*=inv; }
}
static inline void wg_copy3(float* dst, const float* src) {
    dst[0]=src[0]; dst[1]=src[1]; dst[2]=src[2];
}
/** Rodrigues旋转: v_rot = v*cosθ + (k×v)*sinθ + k*(k·v)*(1-cosθ)，k需归一化 */
static void wg_rotate3(const float* axis, float angle_rad, const float* v, float* result) {
    float k[3]; wg_copy3(k, axis); wg_normalize3(k);
    float c = cosf(angle_rad), s = sinf(angle_rad), omc = 1.0f - c;
    float kdv = wg_dot3(k, v);
    float kcv[3]; wg_cross3(k, v, kcv);
    result[0] = v[0]*c + kcv[0]*s + k[0]*kdv*omc;
    result[1] = v[1]*c + kcv[1]*s + k[1]*kdv*omc;
    result[2] = v[2]*c + kcv[2]*s + k[2]*kdv*omc;
}
extern UpperComputerComm upperComputerComm;
extern RobotState robotState;

/*---------------------------------------------------------------------------decision task-----------------------------------------------------------------------------------*/
GimbalControl gimbalControl={0};
const GimbalControl* _gimbalControl = &gimbalControl;
extern DJIGMotorRec yawMotorRec;
extern const DJIGMotorRec* _yawMotorRec;
extern uint8_t angle_error_flag;
extern uint8_t distance_error_flag;
extern void BulletSpeedReceive(void);
ShootControl shootControl={0};            //hxg
const ShootControl* _shootControl = &shootControl;
SmoothFilter MouseFilterX={0};
SmoothFilter MouseFilterY={0};

float pitch_angle_from_match=0;
leastSquareLinear bulletSpeedAdaptation = {
.x = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30},
.count=0
};

static void DecisionInit(void);

static void GimbalInputUpdate(void);
static void ShootInputUpdate(void);

/* 世界系：世界欧拉角(azimuth/elevation) → 机体坐标系向量 f_des_B（前向声明，实现在WorldGimbal区） */
static void WG_WorldAnglesToFdesB(float world_yaw_deg, float world_pitch_deg,
                                   const float* g_B, float* f_des_B_out);

/**
* @brief 机器人目标决策设定，给定各输入值
 */
void DecisionTask(void* argument)
{
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count;	
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._decision_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._decision_task = &this_tick_count;

	DecisionInit();
	BulletKF_Init();//初始化卡尔曼滤波滤波滤波
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	
	while(1)
	{
		/*主任务进程*/
		GimbalInputUpdate();
		ShootInputUpdate();

		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
		vTaskDelayUntil(&current_tick_count, DECISION_TASK_PERIOD_SET);
	}
}

 
/**
 * @brief 决策任务相关初始化
 */
static void DecisionInit(void)
{
	SmoothFilterInitialize(&MouseFilterX,0.7);
	SmoothFilterInitialize(&MouseFilterY,0.7);
}



/**
 * @brief 云台相关输入决策更新
 */
float micro_pitch=0;
int temp_pitch_count=0,last_temp_pitch=0;
float micro_yaw=0;
int temp_yaw_count=0,last_temp_yaww=0;
//int spin_flag;
//int last_spin_flag;
static void GimbalInputUpdate(void)//task1,更新机械限位角度
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
		int mardio;
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


/**
 * @brief 射击相关输入决策更新
 */
float targetspeed[30]={0};
extern DataFromJudge bulletSpeed;
float predict_speed0;
float mardio_speed=15.75;
int16_t fric_speed_left_target , fric_speed_right_target,fric_speed_up_target;
int16_t fric_speed_left_target1 , fric_speed_right_target1,fric_speed_up_target1;
float current_fric_speed =3110;   // 吊射模式弹速; 3500,4650,dansu,4580-16.77//4785
float default_fric_speed = 3110;//常 规模式弹速
float front_fric_speed = 4300;//4550
float back_fric_speed = 4320;
float deltaspeed;
float emergesee;
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
//float current_fric_speed = INITIAL_FRIC_SPEED; 这个之前是注释掉的，看上去下面重复了，估计是调试用的
static void ShootInputUpdate(void)
{
	
	fric_speed_left_target = shootControl.ShootTargetInput.fric_speed_rpm[LEFT];
	fric_speed_right_target = shootControl.ShootTargetInput.fric_speed_rpm[RIGHT];
	fric_speed_up_target = shootControl.ShootTargetInput.fric_speed_rpm[UP];
	fric_speed_left_target1 = shootControl.ShootTargetInput.fric_speed_rpm[LEFT1];
	fric_speed_right_target1 = shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1];
	fric_speed_up_target1 = shootControl.ShootTargetInput.fric_speed_rpm[UP1];

	/*---------------------------------------------------fric--------------------------------------------------------*/
	extern uint8_t shit_dan;
	if(shit_dan)
	{
	BulletSpeedReceive();
	shit_dan=0;
	}
	
	if(_robotState->fric_mode == FRIC_ON)
	{
		//注意符号
		if(_robotState->sniper==SNIPER_ON)
		{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] =+front_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] = -front_fric_speed;             
		shootControl.ShootTargetInput.fric_speed_rpm[UP] = +back_fric_speed;
    shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] =+back_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] = -back_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] = +front_fric_speed;            
		}
		else 
		{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] =+default_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] = -default_fric_speed;            
		shootControl.ShootTargetInput.fric_speed_rpm[UP] = +default_fric_speed;
    shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] =+default_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] = -default_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] = +default_fric_speed;
		}
	}
	else if(_robotState->ctrl_terminal != CONTROL_STOP)                    //HXG1
	{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] = -200;///后右
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] = +200;//后左
		shootControl.ShootTargetInput.fric_speed_rpm[UP] = -200;//前下
    shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] = -200;//前右
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] = +200;//前左
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] = -200;//后上
	}
	else{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[UP] =0;
    shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] =0;
	}
}

/* ====================================================================================================================
 * 世界系云台控制 WorldGimbal 实现
 * 原理：维护虚拟目标指向 f_des_B（机体坐标系B中），用户指令绕大地竖直方向 g_B 和虚拟水平右轴旋转 f_des_B，
 *       再通过阻尼最小二乘IK反解真实yaw/pitch电机角，最终写入 GimbalTargetInput 走现有ADRC/PID控制链路。
 * B系：x=前 y=右 z=下（右手系）
 * ==================================================================================================================== */

/* ---- 全局实例 ---- */
WorldGimbal worldGimbal = {0};
const WorldGimbal* _worldGimbal = &worldGimbal;

/* 外部引用：底盘IMU（485下板传输） */
extern volatile float servant485_pitch_d;
extern volatile float servant485_roll_d;
extern volatile float servant485_yaw_d;

/* 外部引用：yaw角度来自下板485转发（±180°, sniper_on时有效） */
extern volatile float shoot485_yaw_rx_d;

/* 外部引用：pitch电机编码器（上板本地CAN，用于FK/IK） */
extern DJIGMotorRec pitchMotorRec;

/* ===== FK: 真实两轴云台正运动学 ===== */
/* 机械参数（机体坐标系B中） */
static const float WG_AXIS_YAWR_B[3]   = {0.0f, 0.0f, 1.0f};   /* yaw电机轴 = B系z (下) */
static const float WG_AXIS_PITCH0_B[3] = {0.0f, 1.0f, 0.0f};   /* yaw=0时pitch电机轴 = B系y (右) */
static const float WG_POINT0_B[3]      = {1.0f, 0.0f, 0.0f};   /* 零位指向 = B系x (前) */

/**
 * @brief 正运动学：给定电机角，计算云台在机体坐标系中的指向
 * @param q_yaw_rad   yaw电机角 (rad)
 * @param q_pitch_rad pitch电机角 (rad)
 * @param f_out       输出指向单位向量 [3]
 * @param b_out       输出当前pitch轴方向 [3]（可为NULL）
 */
static void WG_ForwardKinematics(float q_yaw_rad, float q_pitch_rad,
                                  float* f_out, float* b_out)
{
    /* 先绕pitch轴转，再绕yaw轴转: f_real = R_yaw * R_pitch * f0 */
    float after_pitch[3];
    wg_rotate3(WG_AXIS_PITCH0_B, q_pitch_rad, WG_POINT0_B, after_pitch);
    wg_rotate3(WG_AXIS_YAWR_B,   q_yaw_rad,   after_pitch, f_out);
    wg_normalize3(f_out);

    if (b_out) {
        /* pitch轴随yaw旋转 */
        wg_rotate3(WG_AXIS_YAWR_B, q_yaw_rad, WG_AXIS_PITCH0_B, b_out);
        wg_normalize3(b_out);
    }
}

/* ===== 编码器 → 电机角转换 ===== */
/* yaw角度：来自下板485转发 shoot485_yaw_rx_d，已换算为±180° */
static inline float WG_GetCurrentYawDeg(void)
{
    return shoot485_yaw_rx_d;
}
/* pitch角度：上板本地LK编码器换算 */
static float WG_PitchEncoderToDeg(uint16_t encoder)
{
    float a = ((float)(encoder) - PITCH_OFFSET_MACHENICAL_ANGLE) * 360.0f / LK_FULL_CIRCLE_MECHENICAL_ANGLE;
    return AngleLimit(a, -180.0f, 180.0f);
}

/* ===== 核心：由底盘IMU计算重力方向在机体坐标系中的表达 g_B ===== */
/**
 * @brief 由底盘roll/pitch计算重力方向 g_B
 * @param chassis_roll_deg  底盘roll (deg, 正=右侧下沉)
 * @param chassis_pitch_deg 底盘pitch (deg, 正=抬头)
 * @param g_B_out           输出 g_B [3]（归一化，方向=大地竖直向下在B系中的表达）
 * @note  g_B = R_body_to_world^T * [0,0,1]^T = [-sin(p), cos(p)*sin(r), cos(p)*cos(r)]
 *        独立于yaw漂移，仅依赖roll/pitch（均有重力加速度参考，不漂）
 */
static void WG_ComputeGravity_B(float chassis_roll_deg, float chassis_pitch_deg, float* g_B_out)
{
    float r = WG_DEG2RAD(chassis_roll_deg);
    float p = WG_DEG2RAD(chassis_pitch_deg);
    float sp = sinf(p), cp = cosf(p);
    float sr = sinf(r), cr = cosf(r);
    g_B_out[0] = -sp;           /* 抬头时重力向后拉 */
    g_B_out[1] =  cp * sr;      /* 右倾时重力向右拉 */
    g_B_out[2] =  cp * cr;      /* 水平时重力沿z_B(下) */
    wg_normalize3(g_B_out);
}

/* ===== 世界系欧拉角 → 机体向量转换 ===== */
/**
 * @brief 将世界系欧拉角（azimuth/elevation）转换为机体坐标系指向向量 f_des_B
 * @param world_yaw_deg   世界系azimuth (deg, 水平面内相对底盘正向投影)
 * @param world_pitch_deg 世界系elevation (deg, 相对水平面的仰角, 正=向上)
 * @param g_B             重力方向在机体坐标系中的表达（归一化, 指向下）
 * @param f_des_B_out     输出 f_des_B [3]（机体坐标系中的单位指向向量）
 * @note  H-frame (水平对齐系): z_H=-g_B(上), x_H=底盘正向水平投影, y_H=z_H×x_H(右)
 *        f_H = [cos(el)*cos(az), cos(el)*sin(az), sin(el)]
 *        f_des_B = f_H[0]*x_H + f_H[1]*y_H + f_H[2]*z_H
 */
static void WG_WorldAnglesToFdesB(float world_yaw_deg, float world_pitch_deg,
                                   const float* g_B, float* f_des_B_out)
{
    float az = WG_DEG2RAD(world_yaw_deg);
    float el = WG_DEG2RAD(world_pitch_deg);

    /* H-frame 基底在B系中的表达 */
    float z_H[3] = {-g_B[0], -g_B[1], -g_B[2]};  /* world UP = -g_B */

    /* x_H = 底盘正向[1,0,0]投影到水平面(⊥g_B) */
    float x_H[3] = {1.0f - g_B[0]*g_B[0], -g_B[0]*g_B[1], -g_B[0]*g_B[2]};
    float x_norm = wg_norm3(x_H);
    if (x_norm > 0.001f) {
        float inv = 1.0f / x_norm;
        x_H[0] *= inv; x_H[1] *= inv; x_H[2] *= inv;
    } else {
        x_H[0] = 1.0f; x_H[1] = 0.0f; x_H[2] = 0.0f;  /* fallback: 底盘水平 */
    }

    /* y_H = z_H × x_H (水平面内的右手方向) */
    float y_H[3];
    wg_cross3(z_H, x_H, y_H);

    /* f_H: azimuth绕z_H转, elevation从水平面抬起 */
    float cel = cosf(el), sel = sinf(el);
    float caz = cosf(az), saz = sinf(az);
    float f_H[3] = {cel * caz, cel * saz, sel};

    /* f_des_B = f_H在B系中的线性组合 */
    f_des_B_out[0] = f_H[0]*x_H[0] + f_H[1]*y_H[0] + f_H[2]*z_H[0];
    f_des_B_out[1] = f_H[0]*x_H[1] + f_H[1]*y_H[1] + f_H[2]*z_H[1];
    f_des_B_out[2] = f_H[0]*x_H[2] + f_H[1]*y_H[2] + f_H[2]*z_H[2];
    wg_normalize3(f_des_B_out);
}

/* ===== 世界系角直接设定 ===== */
/**
 * @brief 用世界系欧拉角直接覆盖虚拟目标指向 f_des_B（用于Q键预设等场景）
 * @param world_yaw_deg   世界系azimuth (deg)
 * @param world_pitch_deg 世界系elevation (deg)
 */
void WorldGimbalSetWorldAngles(WorldGimbal* wg, float world_yaw_deg, float world_pitch_deg)
{
    if (!wg->enable) return;
    WG_WorldAnglesToFdesB(world_yaw_deg, world_pitch_deg,
                          wg->WorldGimbalEstimate.g_B,
                          wg->WorldGimbalTargetInput.f_des_B);
    wg->WorldGimbalTargetInput.init_done = 1;
    wg->WorldGimbalTargetInput.last_right_valid = 0;
}

/* ===== WorldGimbal 初始化 ===== */
void WorldGimbalInit(WorldGimbal* wg)
{
    memset(wg, 0, sizeof(WorldGimbal));
    /* 默认虚拟目标指向前方 */
    wg->WorldGimbalTargetInput.f_des_B[0] = 1.0f;
    wg->WorldGimbalTargetInput.f_des_B[1] = 0.0f;
    wg->WorldGimbalTargetInput.f_des_B[2] = 0.0f;
    wg->WorldGimbalTargetInput.init_done = 0;
    wg->WorldGimbalTargetInput.last_right_valid = 0;
    wg->WorldGimbalEstimate.g_B[2] = 1.0f; /* 默认底盘水平 */
    wg->enable = 0;
}

/**
 * @brief 将虚拟目标对齐到当前真实云台指向（使能世界系时调用，防止跳变）
 */
void WorldGimbalAlignToCurrent(WorldGimbal* wg)
{
    /* 从当前电机角度读取真实指向 */
    float q_yaw_rad   = WG_DEG2RAD(WG_GetCurrentYawDeg());
    float q_pitch_rad = WG_DEG2RAD(WG_PitchEncoderToDeg(pitchMotorRec.mechanical_angle));

    /* FK 得到当前真实指向 */
    WG_ForwardKinematics(q_yaw_rad, q_pitch_rad,
                         wg->WorldGimbalTargetInput.f_des_B, NULL);

    wg->WorldGimbalTargetInput.init_done = 1;
    wg->WorldGimbalTargetInput.last_right_valid = 0;
}

/* ===== WorldGimbal 输入更新（DecisionTask 中调用） ===== */
/**
 * @brief 世界系指令输入：更新虚拟目标指向 f_des_B
 * @param dyaw_deg   世界系yaw增量 (deg, 正=绕重力方向CW)
 * @param dpitch_deg 世界系pitch增量 (deg, 正=抬头)
 */
void WorldGimbalInputUpdate(WorldGimbal* wg, float dyaw_deg, float dpitch_deg)
{
    if (!wg->enable) return;
    if (!wg->WorldGimbalTargetInput.init_done) {
        WorldGimbalAlignToCurrent(wg);
    }

    float* f = wg->WorldGimbalTargetInput.f_des_B;
    const float* g = wg->WorldGimbalEstimate.g_B;

    /* --- 世界系Yaw：绕 g_B（大地竖直方向）旋转 --- */
    if (fabsf(dyaw_deg) > 1e-6f) {
        wg_rotate3(g, WG_DEG2RAD(dyaw_deg), f, f);
        wg_normalize3(f);
    }

    /* --- 世界系Pitch：绕虚拟水平右轴旋转 --- */
    if (fabsf(dpitch_deg) > 1e-6f) {
        /* right_B = normalize(g_B × f_des_B) */
        float right_B[3];
        wg_cross3(g, f, right_B);
        float rnorm = wg_norm3(right_B);

        if (rnorm > 0.001f) {
            /* 正常情况：right轴明确 */
            wg_normalize3(right_B);
            wg_rotate3(right_B, WG_DEG2RAD(dpitch_deg), f, f);
            wg_normalize3(f);
            /* 保存有效的right轴 */
            wg_copy3(wg->WorldGimbalTargetInput.last_right_B, right_B);
            wg->WorldGimbalTargetInput.last_right_valid = 1;
        } else if (wg->WorldGimbalTargetInput.last_right_valid) {
            /* 奇异点附近（f_des接近竖直方向）：复用上一帧right轴 */
            wg_rotate3(wg->WorldGimbalTargetInput.last_right_B,
                       WG_DEG2RAD(dpitch_deg), f, f);
            wg_normalize3(f);
        }
        /* 如果上一帧right也无效，跳过此次pitch（指向完全竖直且无历史） */
    }
}

/* ===== WorldGimbal 观测更新（ControlTask/MotorControlCANSend 中调用） ===== */
void WorldGimbalEstimateUpdate(WorldGimbal* wg)
{
    /* 观测始终运行，不依赖enable状态，方便调试 */

    /* 1. 读取底盘IMU，计算 g_B */
    wg->WorldGimbalEstimate.chassis_roll_deg  = servant485_roll_d;
    wg->WorldGimbalEstimate.chassis_pitch_deg = servant485_pitch_d;
    wg->WorldGimbalEstimate.chassis_yaw_deg   = servant485_yaw_d;

    WG_ComputeGravity_B(wg->WorldGimbalEstimate.chassis_roll_deg,
                        wg->WorldGimbalEstimate.chassis_pitch_deg,
                        wg->WorldGimbalEstimate.g_B);

    /* 2. 从当前电机角度计算FK → f_real_B, b_B */
    float q_yaw_rad   = WG_DEG2RAD(WG_GetCurrentYawDeg());
    float q_pitch_rad = WG_DEG2RAD(WG_PitchEncoderToDeg(pitchMotorRec.mechanical_angle));

    WG_ForwardKinematics(q_yaw_rad, q_pitch_rad,
                         wg->WorldGimbalEstimate.f_real_B,
                         wg->WorldGimbalEstimate.b_B);

    /* 3. 计算指向误差 */
    if (wg->WorldGimbalTargetInput.init_done) {
        float dot_fd_fr = wg_dot3(wg->WorldGimbalTargetInput.f_des_B,
                                  wg->WorldGimbalEstimate.f_real_B);
        if (dot_fd_fr > 1.0f) dot_fd_fr = 1.0f;
        else if (dot_fd_fr < -1.0f) dot_fd_fr = -1.0f;
        wg->WorldGimbalEstimate.angle_error_deg = WG_RAD2DEG(acosf(dot_fd_fr));

        /* 4. 计算世界系欧拉角（从 f_des_B + g_B 反推 elevation/azimuth，供UI/485帧使用） */
        const float* f = wg->WorldGimbalTargetInput.f_des_B;
        const float* g = wg->WorldGimbalEstimate.g_B;

        /* World pitch = elevation above horizontal: sin(elev) = dot(f, -g) = -dot(f,g) */
        float sin_elev = -wg_dot3(f, g);
        if (sin_elev > 1.0f) sin_elev = 1.0f;
        else if (sin_elev < -1.0f) sin_elev = -1.0f;
        wg->WorldGimbalEstimate.world_pitch_deg = WG_RAD2DEG(asinf(sin_elev));

        /* World yaw: f_des_B 投影到水平面（⊥g_B）后相对底盘正向投影的方位角 */
        float dot_fg = wg_dot3(f, g);
        float h[3] = {f[0] - dot_fg * g[0], f[1] - dot_fg * g[1], f[2] - dot_fg * g[2]};
        float h_norm = wg_norm3(h);

        /* 底盘正向 [1,0,0] 投影到水平面 */
        float h_fwd[3] = {1.0f - g[0] * g[0], -g[0] * g[1], -g[0] * g[2]};
        float h_fwd_norm = wg_norm3(h_fwd);

        if (h_norm > 0.001f && h_fwd_norm > 0.001f) {
            float inv_h = 1.0f / h_norm;
            h[0] *= inv_h; h[1] *= inv_h; h[2] *= inv_h;
            float inv_hf = 1.0f / h_fwd_norm;
            h_fwd[0] *= inv_hf; h_fwd[1] *= inv_hf; h_fwd[2] *= inv_hf;

            float cross_hf_h[3];
            wg_cross3(h_fwd, h, cross_hf_h);
            float dot_hf_h = wg_dot3(h_fwd, h);
            if (dot_hf_h > 1.0f) dot_hf_h = 1.0f;
            else if (dot_hf_h < -1.0f) dot_hf_h = -1.0f;
            wg->WorldGimbalEstimate.world_yaw_deg = WG_RAD2DEG(atan2f(wg_dot3(g, cross_hf_h), dot_hf_h));
        } else {
            wg->WorldGimbalEstimate.world_yaw_deg = 0.0f;
        }
    } else {
        wg->WorldGimbalEstimate.world_pitch_deg = 0.0f;
        wg->WorldGimbalEstimate.world_yaw_deg   = 0.0f;
    }
}

/* ===== WorldGimbal IK反解（ControlTask/MotorControlCANSend 中调用） ===== */
/**
 * @brief 阻尼最小二乘IK：从 f_des_B 反解真实电机角
 * @note  Jacobian: Jy = a_B × f_real, Jp = b_B × f_real
 *        误差: rotation_error = f_real × f_des, e = rotation_error × f_real
 *        求解: (J^T J + λI)·dq = J^T e  (2×2线性系统)
 */
void WorldGimbalIKSolve(WorldGimbal* wg)
{
    if (!wg->enable || !wg->WorldGimbalTargetInput.init_done) return;

    const float lambda = WORLDGIMBAL_IK_LAMBDA;
    const float max_step = WORLDGIMBAL_IK_MAX_STEP_RAD;
    const float converge_thresh = WORLDGIMBAL_IK_CONVERGE_RAD;

    /* 从当前编码器读取实时电机角作为IK初始值 */
    float q_yaw   = WG_DEG2RAD(WG_GetCurrentYawDeg());
    float q_pitch = WG_DEG2RAD(WG_PitchEncoderToDeg(pitchMotorRec.mechanical_angle));

    const float* f_des = wg->WorldGimbalTargetInput.f_des_B;
    const float* a_B   = WG_AXIS_YAWR_B;

    uint8_t iter;
    for (iter = 0; iter < WORLDGIMBAL_IK_MAX_ITERS; iter++) {
        /* FK: 计算当前指向和pitch轴 */
        float f_real[3], b_B[3];
        WG_ForwardKinematics(q_yaw, q_pitch, f_real, b_B);

        /* 误差: rotation_error = f_real × f_des, e = rotation_error × f_real */
        float rot_err[3], e[3];
        wg_cross3(f_real, f_des, rot_err);
        /* 如果 f_real ≈ f_des，rot_err ≈ 0，提前收敛 */
        float err_norm = wg_norm3(rot_err);
        if (err_norm < converge_thresh) break;

        wg_cross3(rot_err, f_real, e);

        /* Jacobian列 */
        float Jy[3], Jp[3];
        wg_cross3(a_B, f_real, Jy);   /* d(f_real)/d(q_yaw) */
        wg_cross3(b_B, f_real, Jp);   /* d(f_real)/d(q_pitch) */

        /* 2×2 正规方程: A·dq = g */
        float A00 = wg_dot3(Jy, Jy) + lambda;
        float A01 = wg_dot3(Jy, Jp);
        float A11 = wg_dot3(Jp, Jp) + lambda;
        float g0  = wg_dot3(Jy, e);
        float g1  = wg_dot3(Jp, e);

        float det = A00 * A11 - A01 * A01;
        if (fabsf(det) < 1e-12f) break; /* 奇异，放弃本周期 */

        float dq_yaw   = (A11 * g0 - A01 * g1) / det;
        float dq_pitch = (A00 * g1 - A01 * g0) / det;

        /* 步长限制 */
        if (dq_yaw   >  max_step) dq_yaw   =  max_step;
        if (dq_yaw   < -max_step) dq_yaw   = -max_step;
        if (dq_pitch >  max_step) dq_pitch =  max_step;
        if (dq_pitch < -max_step) dq_pitch = -max_step;

        q_yaw   += dq_yaw;
        q_pitch += dq_pitch;
    }

    /* 输出 */
    wg->WorldGimbalControl.q_yaw_cmd_rad   = q_yaw;
    wg->WorldGimbalControl.q_pitch_cmd_rad = q_pitch;
    wg->WorldGimbalControl.q_yaw_cmd_deg   = WG_RAD2DEG(q_yaw);
    wg->WorldGimbalControl.q_pitch_cmd_deg = WG_RAD2DEG(q_pitch);
    wg->WorldGimbalControl.converged       = (iter < WORLDGIMBAL_IK_MAX_ITERS) ? 1 : 0;
    wg->WorldGimbalControl.iters_used      = iter;
}

/* ===== 将IK结果写入 GimbalTargetInput，走现有控制链路 ===== */
void WorldGimbalApplyToTargets(WorldGimbal* wg)
{
    if (!wg->enable) return;
    if (!wg->WorldGimbalTargetInput.init_done) return;  /* IK还未就绪，不写入 */

    /* Yaw：写入目标角（485转发给下板DMJ4310） */
    gimbalControl.GimbalTargetInput.yaw_angle_d = wg->WorldGimbalControl.q_yaw_cmd_deg;

    /* Pitch：写入目标角（后续走LTD+ADRC闭环） */
    gimbalControl.GimbalTargetInput.pitch_angle_d = wg->WorldGimbalControl.q_pitch_cmd_deg;
}
/*---------------------------------------------------------------------------control task-----------------------------------------------------------------------------------*/
static void ControlInit(void);

static void GimbalEstimateUpdate(void);
static void ShootEstimateUpdate(void);

static void	GimbalControlUpdate(void);
static void ShootControlUpdate(void);
/**
 * @brief 控制任务，完成各电机控制闭环
 */

void ControlTask(void* argument)
{
	/*任务周期相关计算，并将其绑定到TaskMonitor相关指针中*/
	static uint32_t last_tick_count, current_tick_count, this_tick_count;	
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._control_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._control_task = &this_tick_count;
	
	ControlInit();
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	vTaskDelay(400);
	while(1)
	{

		/*任务主进程*/
		//观测
		GimbalEstimateUpdate();
		ShootEstimateUpdate();

		//闭环
		GimbalControlUpdate();
		ShootControlUpdate();
		
		//发送指令
		MotorControlCANSend();
		
		
		

		/*任务状态更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
		vTaskDelayUntil(&current_tick_count, CONTROL_TASK_PERIOD_SET);
	}
}

/**
 * @brief 控制执行机构相关初始化
 */
float kpfric=17.5;       //hxgpid
float kdfric=12.5;
static void ControlInit(void)
{
	/* 世界系云台控制初始化 */
	WorldGimbalInit(&worldGimbal);

	//云台电机
  //PIDInitialize(&(gimbalControl.GimbalMotorControl.yaw_speed_pid ), 350, 0, 25, 0, GM6020_MAX_OUTPUT_VOLTAGE);
	//PIDInitialize(&(gimbalControl.GimbalMotorControl.yaw_angle_pid ), 350, 0, 10, 0, GM6020_MAX_OUTPUT_VOLTAGE);
	float pitch_angle_offset_d = 4.9;
	gimbalControl.GimbalTargetInput.pitch_angle_d = pitch_angle_offset_d;
	float h_temp = CONTROL_TASK_PERIOD_SET / 1000.0;
	float td_init_val[3] = {300, h_temp, 2};
	
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
	float pitch_lesf_init_val[5] = {0.04f, 11, 0, 200, 1000};		
	if(1){		
			// --- 1. 定义你的核心 ESO 参数 ---
	const float yaw_eso_wo = 100.0f;    // <<<<<< 你的观测器带宽，从这里开始调！(建议从100-200开始)
	const float yaw_eso_b0 = 100.0f;   // <<<<<< 你的控制增益b0，从这里开始调！(需要物理估算)
	const float yaw_z3_limit = 15000.0f; // 扰动估计限幅
	// --- 2. 在定义数组时，直接使用上面的参数进行计算 ---
	float yaw_eso_init_val[6] = {
			h_temp,                                     // [0] h: 控制周期
			yaw_eso_b0,                                 // [1] b0: 控制增益
			3.0f * yaw_eso_wo,                          // [2] beta_01 = 3*wo
			3.0f * yaw_eso_wo * yaw_eso_wo,             // [3] beta_02 = 3*wo^2
			1.0f * yaw_eso_wo * yaw_eso_wo * yaw_eso_wo,// [4] beta_03 = 1*wo^3
			yaw_z3_limit                                // [5] z3_limit
	};
	float yaw_lesf_init_val[5] = {0.05f, 100, 1, 100, 1000};//
	gimbalControl.GimbalMotorControl.pitch_LTD.ki1=0.02;//还有一个ki   0.01
	gimbalControl.GimbalMotorControl.pitch_LTD.error_sum=0;
	gimbalControl.GimbalMotorControl.pitch_LTD.error_sum_max=90;            //80;
	gimbalControl.GimbalMotorControl.pitch_LTD.lv_bo=0.3;
	
	//yaw
	LTDInitialize(&gimbalControl.GimbalMotorControl.yaw_LTD, 30, 0.003, -180, 180);
	LTDPIDInitialize(&gimbalControl.GimbalMotorControl.yaw_LTD_pid,10,200,30000,30000);  
	gimbalControl.GimbalMotorControl.yaw_LTD.ki1=0;
	gimbalControl.GimbalMotorControl.yaw_LTD.error_sum=0;
	gimbalControl.GimbalMotorControl.yaw_LTD.error_sum_max=100;
	gimbalControl.GimbalMotorControl.yaw_LTD.lv_bo=1;                   //hxg
	//摩擦轮
	PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[LEFT]),  kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
	PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[RIGHT]), kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
	PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[UP]), 	 kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
	PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[LEFT1]),  kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
	PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[RIGHT1]), kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
	PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[UP1]), 	 kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
}
}
float see_error1,see_error2,see_error3;
float w_d2=0;
float k_ff=2;
int temp_yaw=0,last_temp_yaw=0;
//float 
float wd3=3,wd3p=30;
/*新云台控制函数*/
extern DJIGMotorRec smallpitchMotorRec;
extern Pose gimbalPose;
int finish_flag=0;
int target_torque=0;
float w_d;
float last_compensation;
float compensation;
float gravity_compensation;
float fric_compensation;
int last_u;
int fl_u;
int shoot_flag;
int shoot_cnt;
float sniper_pitch_angle;
float angle_control;
float lowpass_pitch=0;
extern int64_t circle_angle;
extern float pitch_angle_from_match;
void ALLHighFreqCal(void)
{
	extern uint8_t shit_delay_count;

	/* Estimate始终运行；IK+Apply仅在state_task使能enable时执行 */
	WorldGimbalEstimateUpdate(&worldGimbal);
	WorldGimbalIKSolve(&worldGimbal);
	WorldGimbalApplyToTargets(&worldGimbal);

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
		sniper_pitch_angle=(_pitchMotorRec->mechanical_angle - PITCH_OFFSET_MACHENICAL_ANGLE) * 360.0f /LK_FULL_CIRCLE_MECHENICAL_ANGLE;
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
		sniper_pitch_angle=(_pitchMotorRec->mechanical_angle - PITCH_OFFSET_MACHENICAL_ANGLE) * 360.0f /LK_FULL_CIRCLE_MECHENICAL_ANGLE;
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
		
		gimbalControl.GimbalMotorControl.yaw_LTD.error_sum=0;
		gimbalControl.GimbalMotorControl.pitch_target_output = gimbalControl.GimbalMotorControl.yaw_target_output = 0;
		gimbalControl.GimbalMotorControl.small_pitch_target_output = gimbalControl.GimbalMotorControl.small_pitch_target_output = 0;
	}
}
/**
 * @brief 云台相关观测数据更新
 */
static void GimbalEstimateUpdate(void)
{
}

/// @brief 云台观测位姿更新，该更新周期与imu_task任务更新位姿周期一致，故需外部调用该更新函数
/// @param pitch_angle 观测当前pitch轴角，单位degree
/// @param pitch_angle_w 观测当前pitch转角方向角速度，单位rad/s
/// @param yaw_angle  观测当前yaw轴角，单位degree
/// @param yaw_angle_w 观测当前yaw转角方向角速度，单位rad/s
/// @note 角速度仍有高频噪声，需经过平滑滤波
int temp_angle_machine=0;
//extern uint8_t lastRobotState.lens;
float yaw_angle_from_mach=0;
uint8_t shit_delay_count=0;
float shit_temp_pitch_comp=0;
void GimbalPoseUpdate(float pitch_angle, float pitch_angle_w, float yaw_angle, float yaw_angle_w,float roll_angle,float roll_angle_w)
{
	gimbalControl.GimbalEstimate.roll_angle_d=roll_angle;
	gimbalControl.GimbalEstimate.roll_angular_velocity_dps=roll_angle_w;
	yaw_angle_from_mach=AngleLimit(((float)yawMotorRec.mechanical_angle+temp_angle_machine-CHASSIS_FORWARD_YAW_MACHENICAL_ANGLE)*360/8192,-180,180);
	pitch_angle_from_match= (_pitchMotorRec->mechanical_angle - PITCH_OFFSET_MACHENICAL_ANGLE) * 360.0f /LK_FULL_CIRCLE_MECHENICAL_ANGLE;//符号注意，记得改这玩意
	pitch_angle_from_match= AngleLimit(pitch_angle_from_match, -180, 180);
	//-18.27 -17.28 
	/*为什么之前用的是编码器，因为IMU会漂移*/
	/*吊射模式防止漂移用编码器，普通模式为了跟随和自瞄用IMU
	这个shit_delay用来防止发送太快没来得及更改target，后来人有更好方法请删掉，比如什么freerrtos*/
	/*IMU pitch漂移，解决了又重现了，有除了滤波之外的东西，比如IMU本体问题*/

	if(_robotState->sniper==SNIPER_ON){ //task5,机械角问题以后再说
			gimbalControl.GimbalEstimate.yaw_angle_d=yaw_angle_from_mach;
			if(shit_delay_count<200)
			shit_delay_count++;
		if(lastRobotState.lens==SNIPER_OFF && (_robotState->sniper==SNIPER_ON)){
			shit_delay_count=0;
			gimbalControl.GimbalTargetInput.yaw_angle_d=yaw_angle_from_mach;
			lastRobotState.lens=SNIPER_ON;
			gimbalControl.GimbalTargetInput.yaw_angle_d=gimbalControl.GimbalEstimate.yaw_angle_d;
		}
		}int mardio;
		
		if(_robotState->sniper==SNIPER_OFF){
			if(shit_delay_count<200)
			shit_delay_count++;
			gimbalControl.GimbalEstimate.yaw_angle_d = yaw_angle;
			if(lastRobotState.lens==SNIPER_ON && _robotState->sniper==SNIPER_OFF){
				shit_delay_count=0;
				gimbalControl.GimbalTargetInput.yaw_angle_d	= yaw_angle;
				lastRobotState.lens=SNIPER_OFF;
				gimbalControl.GimbalTargetInput.yaw_angle_d=gimbalControl.GimbalEstimate.yaw_angle_d;
			}
		}
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
/**
 * @brief 发射相关观测更新
 */
static void ShootEstimateUpdate(void)
{
	/* 拨盘已移除，保留空壳 */
}
/**
 * @brief 云台闭环控制，难绷云台控制里面没有云台了，O(∩_∩)O哈哈~
 */ 
int duoji=100000;
int duoji2=52300;
float K1=0,K2=0,K3=0;//妈的
static void GimbalControlUpdate(void)
{
	/*望远瞄具舵机决策+控制+发送*/
	
	//if(CONTROL_STOP != _robotState->ctrl_terminal)
	//{
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		if(_robotState->lens==LENS_OFF)//调试用
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,duoji);
		if(_robotState->lens==LENS_ON)
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,duoji2);



	#if defined GIMBAL_OFF
		gimbalControl.GimbalMotorControl.pitch_target_output = gimbalControl.GimbalMotorControl.yaw_target_output = 0;
	#endif
	
	if(CONTROL_STOP == _robotState->ctrl_terminal)
	{
		gimbalControl.GimbalMotorControl.pitch_target_output = gimbalControl.GimbalMotorControl.small_pitch_target_output=0;
		gimbalControl.GimbalMotorControl.yaw_target_output = 0;
	}	
}

/**
 * @brief 发射闭环控制
 */ 
static void ShootControlUpdate(void)   
{
	//摩擦轮
	shootControl.ShootMotorControl.fric_target_output[LEFT] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[LEFT], \
																		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] - _fricMotorRec[LEFT].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[RIGHT] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[RIGHT], \
																		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] - _fricMotorRec[RIGHT].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[UP] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[UP], \
																		shootControl.ShootTargetInput.fric_speed_rpm[UP] - _fricMotorRec[UP].mechanical_speed_rpm);
	
    shootControl.ShootMotorControl.fric_target_output[LEFT1] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[LEFT1], \
																		shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] - _fricMotorRec[LEFT1].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[RIGHT1] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[RIGHT1], \
																		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] - _fricMotorRec[RIGHT1].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[UP1] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[UP1], \
																		shootControl.ShootTargetInput.fric_speed_rpm[UP1] - _fricMotorRec[UP1].mechanical_speed_rpm);
}

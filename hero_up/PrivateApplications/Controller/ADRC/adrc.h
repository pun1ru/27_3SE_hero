/**
 * @file    adrc.h
 * @brief   自抗扰控制(ADRC) —— 跟踪微分器 / 扩张状态观测器 / 误差组合器
 * @note    微分器种类与二阶误差速度来源均通过函数指针绑定，上层只调用统一入口
 */

#ifndef _ADRC_H_
#define _ADRC_H_

#include "stdint.h"

/*--------------------------------------------------- 数值与配置宏 ---------------------------------------------------*/

/* Fal 非线性函数幂次：z2 观测支路取 0.5，z3 扰动支路取 0.25 */
#define ESO_FAL_ALPHA_Z2 (0.5f)
#define ESO_FAL_ALPHA_Z3 (0.25f)

/* Fal 线性区宽度相对积分步长的倍率，1 倍步长为默认整定值 */
#define ESO_FAL_DELTA_GAIN_UNIT (1.0f)

/* yaw 观测器 z2 支路整定为 10 倍步长，线性区更宽以抑制观测抖动 */
#define YAW_ESO_FAL_DELTA_GAIN_Z2 (10.0f)

/* z1/z2 边界限幅使能：未设定边界的轴必须关闭，否则会被限幅到 0 */
#define ESO_STATE_LIMIT_DISABLE (0U)
#define ESO_STATE_LIMIT_ENABLE (1U)

/* 扰动补偿权重：1.0 表示按 z3 全量补偿 */
#define ADRC_Z3_GAIN_FULL (1.0f)

/*----------------------------------------------------- 类型定义 -----------------------------------------------------*/

/**
 * @brief TD跟踪微分器实现(Tracking Differentiator)
 * @note 由于该过滤器存在严重的相角滞后，一般该微分器适用于处理输入信号而不适合处理反馈信号
 */
typedef struct
{
    float input;
    /*信号一阶及二阶跟踪量*/
    float x1;
    float x2;

    /*系统参数*/
    float r;   // 速度因子--值越大，逼近速度越快，信号更趋近于真实曲线，有棱有角，取决于受控对象能否承受
    float h;   // 积分步长，即运行周期
    uint8_t N; // 积分步长扩大系数，用于增大h，减少震荡，一般N*h增大，能适当抑制噪声
    /*处理周期信号的上下限*/
    float min;
    float max;
} TD;

/**
 * @brief LTD线性跟踪微分器：比 TD 高一阶，输出更平滑，步长由 DWT 实测给出
 */
typedef struct
{
    float input;
    /*信号一阶及二阶跟踪量*/
    float x1;
    float x2;

    /*系统参数*/
    float r; // 速度因子--值越大，逼近速度越快，信号更趋近于真实曲线，有棱有角，取决于受控对象能否承受
    float h; // 积分步长，即运行周期
    float min;
    float max;
    uint32_t cnt;

    /*一些工程上的配置参数，只看LTD可以无视，对于本车结合使用请*/
    float kp1;
    float kp2;
    float kd1;
    float kd2;
    float ki1;
    float ki2;
    float error_sum;
    float error_sum_max;
    float lv_bo;
    float out_put_limit;
} LTD;

/**
 * @brief LTD 配套的工程 PID 参数组
 */
typedef struct
{
    float kp;
    float _pitchkd;
    float w_d_limit;
    float p_output_limit;
} LTDPID;

/**
 * @brief 微分器状态推进回调
 * @param differentiator 实际微分器实例，TD 或 LTD
 * @param target         目标量
 * @retval void
 */
typedef void (*TrackDiffUpdateFunc)(void *differentiator, float target);

/**
 * @brief 微分器状态读取回调
 * @param differentiator 实际微分器实例
 * @param x1             一阶跟踪量输出
 * @param x2             二阶跟踪量输出
 * @retval void
 */
typedef void (*TrackDiffStateFunc)(const void *differentiator, float *x1, float *x2);

/**
 * @brief 微分器统一句柄
 * @note  TD 与 LTD 各自绑定回调后收口到同一调用入口，ADRC 内部不再区分微分器种类
 */
typedef struct
{
    void *handle;                 // 指向实际的 TD 或 LTD 实例
    TrackDiffUpdateFunc update;   // 状态推进回调
    TrackDiffStateFunc get_state; // 状态读取回调
} TrackDiff;

/**
 * @brief 拓张状态观测器，估计扰动
 */
typedef struct
{
    /*系统观测量*/
    float z1;
    float z2;
    float z3;

    /*系统参数--根据带宽法整定参数，取beta_01 = 1/h beta_02 = 1/(3*h*h) beta_03=1/(20*h*h*h)*/
    float h;       // 积分步长
    float b;       // 控制量系数，控制量z2计算式叠加了一个b*u的项，u即为控制量
    float beta_01; // 估计一阶量z1时非线性函数扩大系数
    float beta_02; // 估计二阶量z2时非线性函数扩大系数
    float beta_03; // 估计三阶量z3时非线性函数扩大系数

    /*处理周期信号的上下限及限幅*/
    float z3_limit;
    float min;
    float max;
    float z1_min;
    float z1_max;
    float z2_min;
    float z2_max;

    /*逐轴整定接口--yaw 与 pitch 的观测器差异全部收口到这三个参数*/
    float fal_delta_gain_z2;    // z2 支路 Fal 线性区宽度相对步长的倍率
    float fal_delta_gain_z3;    // z3 支路 Fal 线性区宽度相对步长的倍率
    uint8_t state_limit_enable; // z1/z2 边界限幅使能，未设定边界时必须关闭
} ESO;

/**
 * @brief 误差输出组合器
 */
typedef struct
{
    float e_0; // 误差累计
    float e_1;
    float e_2;
    float output; // 当前输出

    float k_0; // 积分项扩大系数
    float k_1; // 误差一阶项扩大系数
    float k_2; // 误差二阶项扩大系数

    float e_0_max;      // 误差累计上限
    float output_limit; // 输出上限
} ESF;

struct ADRCStruct;

/**
 * @brief 二阶误差参考速度来源回调
 * @param adrc ADRC 结构体
 * @retval float 参与 LESF 组合的速度值
 * @note  yaw 取 ESO 观测速度 z2，pitch 取外部实测角速度，绑定不同回调即可切换
 */
typedef float (*VelocityRefFunc)(const struct ADRCStruct *adrc);

/**
 * @brief 自抗扰控制器
 * @note  td/eso/esf 顺序与 LADRCInitialize 的入参数组一一对应，严禁调整
 */
typedef struct ADRCStruct {
    TD td;
    ESO eso;
    ESF esf;

    TrackDiff track_diff;         // 微分器统一句柄，默认绑定内部 td
    VelocityRefFunc velocity_ref; // 二阶误差速度来源，默认取 eso.z2
    float z3_gain;                // 扰动补偿权重，ADRC_Z3_GAIN_FULL 为全量补偿

    float u;         // 补偿后控制量
    float u_0;       // 补偿前控制量
    float limit_max; // 处理周期信号上限
    float limit_min; // 处理周期信号下限
} ADRC;

/*----------------------------------------------------- 函数声明 -----------------------------------------------------*/

/* 非线性函数 */
float Fal(float e, float alpha, float delta);

/* TD 跟踪微分器 */
void TDInitialize(TD *td, float r, float h0, float N, float min, float max);
void TDSetParam(TD *td, float r, float h0, float N, float min, float max);
void TD_Reset(TD *td, float x1);
void TDUpdate(TD *td, float target);

/* LTD 线性跟踪微分器 */
void LTDInitialize(LTD *ltd, float r, float h, float min, float max);
void LTDSetParam(LTD *ltd, float r, float h, float min, float max);
void LTD_Reset(LTD *ltd, float x1);
void LTDUpdate(LTD *ltd, float target);
void LTDUpdateNoLimit(LTD *ltd, float target);
void LTDPIDInitialize(LTDPID *ltdpid, float kp, float _pitchkd, float w_d_limit, float p_output_limit);

/* 微分器统一句柄：绑定后由 TrackDiffUpdate/TrackDiffGetState 统一驱动 */
void TrackDiffBindTD(TrackDiff *track_diff, TD *td);
void TrackDiffBindLTD(TrackDiff *track_diff, LTD *ltd);
void TrackDiffUpdate(TrackDiff *track_diff, float target);
void TrackDiffGetState(const TrackDiff *track_diff, float *x1, float *x2);

/* ESO 扩张状态观测器 */
void ESOInitialize(ESO *eso, float h, float b, float b_01, float b_02, float b_03, float z3_limit, float min, float max);
void ESOSetFalTuning(ESO *eso, float fal_delta_gain_z2, float fal_delta_gain_z3);
void ESOSetStateLimit(ESO *eso, float z1_min, float z1_max, float z2_min, float z2_max);
void ESOUpdate(ESO *eso, float feedback, float control_val);
void ESO_Reset(ESO *eso, float z1);

/* ESF 线性组合器（线性组合等同于PID） */
void LESFInitialize(ESF *esf, float k_0, float k_1, float k_2, float e_0_max, float output_limit);
float LESFUpdate(ESF *esf, float e_1, float e_2);

/* ADRC 本体 */
void LADRCInitialize(ADRC *adrc, float *td_init_val, float *lesf_init_val, float *eso_init_val, float min, float max);
void ADRCBindTrackDiffLTD(ADRC *adrc, LTD *ltd);
void ADRCBindVelocityRef(ADRC *adrc, VelocityRefFunc velocity_ref);
float ADRCVelocityFromObserver(const ADRC *adrc);
float ADRCVelocityFromPitchEstimate(const ADRC *adrc);
void ADRCUpdate(ADRC *adrc, float target, float feedback);

/* 兼容入口：保持既有调用点签名，内部统一走 ADRCUpdate */
void LADRCUpdate(ADRC *adrc, float target, float feedback);
void YawLADRCUpdate(ADRC *adrc, float target, float feedback);
void LADRCUpdateV2(ADRC *adrc, float target, float feedback, float actual_velocity, float z3_gain);
void LTDADRCUpdate(ADRC *adrc, LTD *ltd, float target, float feedback);

#endif

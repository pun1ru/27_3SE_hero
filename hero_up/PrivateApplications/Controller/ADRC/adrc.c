/**
 * @file    adrc.c
 * @brief   自抗扰控制(ADRC)实现 —— 跟踪微分器 / 扩张状态观测器 / 误差组合器
 * @note    yaw 与 pitch 的观测器差异由 ESO 内的整定字段承载，微分器种类与
 *          二阶误差速度来源由函数指针绑定，上层统一调用 ADRCUpdate()
 */

#include "math.h"
#include "bsp_dwt.h"
#include "adrc.h"
#include "algorism.h"
#include "gimbalControl.h"

/*--------------------------------------------------- 非线性基础函数 ---------------------------------------------------*/

/**
 * @brief  fhan最速综合函数
 * @param  x1 一阶跟踪误差
 * @param  x2 二阶跟踪量
 * @param  r  速度因子
 * @param  h  积分步长
 * @retval float 二阶量增量
 * @note   根据输入信号的大小和变化率自适应地调整输出信号的幅值和频率，使得输出信号
 *         能够最快地跟踪输入信号，同时抑制噪声和干扰的影响
 */
static float fhan(float x1, float x2, float r, float h){
    float d, a0, y, a1, a2, sy, a, sa;
    d = r * square(h);
    a0 = h * x2;
    y = x1 + a0;
    a1 = sqrt(d * (d + 8 * fabs(y)));
    a2 = a0 + fsgn(y) * (a1 - d) / 2.0f;
    sy = (fsgn(y + d) - fsgn(y - d)) / 2.0f;
    a = (a0 + y - a2) * sy + a2;
    sa = (fsgn(a + d) - fsgn(a - d)) / 2.0f;
    return -r * (a / (1.0f * d) - fsgn(a)) * sa - r * fsgn(a);
}

/**
 * @brief  非线性fal函数
 * @param  e     误差
 * @param  alpha 幂次
 * @param  delta 线性区宽度
 * @retval float 非线性映射结果
 * @note   y = e / delta^(1-alpha) when fabs(e) < fabs(delta)
 *         y = sign(e) * fabs(e)^alpha when fabs(e) > fabs(delta)
 *         误差小采用线性部分，误差大展现幂函数特性
 */
float Fal(float e, float alpha, float delta){
    if(fabs(e) <= fabs(delta)){
        return e / powf(fabs(delta), 1 - alpha);
    } else {
        return fsgn(e) * powf(fabs(e), alpha);
    }
}

/*----------------------------------------------------- LTD 微分器 -----------------------------------------------------*/

/**
 * @brief  LTD线性跟踪微分器初始化
 * @param  ltd LTD结构体
 * @param  r   速度因子
 * @param  h   积分步长
 * @param  min 周期信号下限
 * @param  max 周期信号上限
 * @retval void
 * @note   若处理非周期信号，设置上下限均相等
 */
void LTDInitialize(LTD *ltd, float r, float h, float min, float max){
    ltd->r = r;
    ltd->min = min;
    ltd->max = max;
    ltd->h = h;
}

/**
 * @brief  LTD参数在线设置
 * @param  ltd LTD结构体
 * @param  r   速度因子
 * @param  h   积分步长
 * @param  min 周期信号下限
 * @param  max 周期信号上限
 * @retval void
 */
void LTDSetParam(LTD *ltd, float r, float h, float min, float max){
    ltd->r = r;
    ltd->h = h;
    ltd->min = min;
    ltd->max = max;
}

/**
 * @brief  LTD状态复位：直接跟踪到给定x1，速度与积分清零
 * @param  ltd LTD结构体
 * @param  x1  复位目标值
 * @retval void
 */
void LTD_Reset(LTD *ltd, float x1){
    ltd->x1 = x1;
    ltd->x2 = 0.0f;
    ltd->error_sum = 0.0f;
}

/**
 * @brief  LTD微分器计算
 * @param  ltd    LTD结构体
 * @param  target 目标量
 * @retval void
 * @note   dx2/dt = -2*r*x2 - r^2*(x1 - target)，等价于弹簧-阻尼系统动力学模型；
 *         r 调得过大时状态可能发散为 NAN，故保留数值安全检查
 */
void LTDUpdate(LTD *ltd, float target){
    float x1_delta = ltd->x1 - target;
    ltd->h = DWT_GetDeltaT(&ltd->cnt);
    if(ltd->min != ltd->max){
        x1_delta = AngleLimit(x1_delta, ltd->min, ltd->max);
    }

    /* 状态更新（二阶系统积分） */
    ltd->x1 += ltd->h * ltd->x2;
    ltd->x2 += ltd->h * (-2 * ltd->r * ltd->x2 - ltd->r * ltd->r * x1_delta);

    /* 数值安全检查：发散时把状态强行拉回目标值并清速度 */
    if(!isfinite(ltd->x1) || !isfinite(ltd->x2)){
        ltd->x1 = target;
        ltd->x2 = 0.0f;
    }

    if(ltd->min != ltd->max){
        ltd->x1 = AngleLimit(ltd->x1, ltd->min, ltd->max);
    }
}

/**
 * @brief  LTD微分器计算（不做周期限幅）
 * @param  ltd    LTD结构体
 * @param  target 目标量
 * @retval void
 * @note   与 LTDUpdate 实现一致，但不对 x1 做周期性环绕处理，
 *         适用于非周期角度信号；保留数值发散保护
 */
void LTDUpdateNoLimit(LTD *ltd, float target){
    float x1_delta = ltd->x1 - target;
    ltd->h = DWT_GetDeltaT(&ltd->cnt);

    /* 状态更新（二阶系统积分） */
    ltd->x1 += ltd->h * ltd->x2;
    ltd->x2 += ltd->h * (-2.0f * ltd->r * ltd->x2 - ltd->r * ltd->r * x1_delta);

    /* 数值安全检查：发散时复位到目标并清速度 */
    if(!isfinite(ltd->x1) || !isfinite(ltd->x2)){
        ltd->x1 = target;
        ltd->x2 = 0.0f;
    }
}

/**
 * @brief  LTD配套工程PID参数初始化
 * @param  ltdpid          LTDPID结构体
 * @param  kp              位置项系数
 * @param  _pitchkd        pitch微分项系数
 * @param  w_d_limit       期望角速度限幅
 * @param  p_output_limit  位置环输出限幅
 * @retval void
 */
void LTDPIDInitialize(LTDPID *ltdpid, float kp, float _pitchkd, float w_d_limit, float p_output_limit){
    ltdpid->kp = kp;
    ltdpid->_pitchkd = _pitchkd;
    ltdpid->w_d_limit = w_d_limit;
    ltdpid->p_output_limit = p_output_limit;
}

/*----------------------------------------------------- TD 微分器 -----------------------------------------------------*/

/**
 * @brief  TD微分器初始化
 * @param  td  TD结构体
 * @param  r   速度因子
 * @param  h0  积分步长
 * @param  N   积分步长扩大系数
 * @param  min 周期信号下限
 * @param  max 周期信号上限
 * @retval void
 * @note   若处理非周期信号，设置上下限均为0
 */
void TDInitialize(TD *td, float r, float h0, float N, float min, float max){
    td->h = h0 * N;
    td->r = r;
    td->max = max;
    td->min = min;
}

/**
 * @brief  TD参数在线设置
 * @param  td  TD结构体
 * @param  r   速度因子
 * @param  h0  积分步长
 * @param  N   积分步长扩大系数
 * @param  min 周期信号下限
 * @param  max 周期信号上限
 * @retval void
 */
void TDSetParam(TD *td, float r, float h0, float N, float min, float max){
    td->h = h0 * N;
    td->r = r;
    td->max = max;
    td->min = min;
}

/**
 * @brief  TD状态复位
 * @param  td TD结构体
 * @param  x1 复位目标值
 * @retval void
 */
void TD_Reset(TD *td, float x1){
    td->x1 = x1;
    td->x2 = 0.0f;
}

/**
 * @brief  TD微分器计算
 * @param  td     TD结构体
 * @param  target 目标量
 * @retval void
 */
void TDUpdate(TD *td, float target){
    float x1_delta = td->x1 - target;

    if(td->min != td->max){
        x1_delta = AngleLimit(x1_delta, td->min, td->max);
    }

    td->x1 += td->h * td->x2;
    td->x2 += td->h * fhan(x1_delta, td->x2, td->r, td->h);

    if(td->min != td->max){
        td->x1 = AngleLimit(td->x1, td->min, td->max);
    }
}

/*------------------------------------------------ 微分器统一句柄（函数指针） ------------------------------------------------*/

/**
 * @brief  TD 推进回调
 * @param  differentiator TD实例
 * @param  target         目标量
 * @retval void
 */
static void trackDiffTDUpdate(void *differentiator, float target){
    TDUpdate((TD *)differentiator, target);
}

/**
 * @brief  TD 状态读取回调
 * @param  differentiator TD实例
 * @param  x1             一阶跟踪量输出
 * @param  x2             二阶跟踪量输出
 * @retval void
 */
static void trackDiffTDGetState(const void *differentiator, float *x1, float *x2){
    const TD *td = (const TD *)differentiator;
    *x1 = td->x1;
    *x2 = td->x2;
}

/**
 * @brief  LTD 推进回调
 * @param  differentiator LTD实例
 * @param  target         目标量
 * @retval void
 */
static void trackDiffLTDUpdate(void *differentiator, float target){
    LTDUpdate((LTD *)differentiator, target);
}

/**
 * @brief  LTD 状态读取回调
 * @param  differentiator LTD实例
 * @param  x1             一阶跟踪量输出
 * @param  x2             二阶跟踪量输出
 * @retval void
 */
static void trackDiffLTDGetState(const void *differentiator, float *x1, float *x2){
    const LTD *ltd = (const LTD *)differentiator;
    *x1 = ltd->x1;
    *x2 = ltd->x2;
}

/**
 * @brief  句柄绑定到 TD 微分器
 * @param  track_diff 微分器句柄
 * @param  td         TD实例
 * @retval void
 */
void TrackDiffBindTD(TrackDiff *track_diff, TD *td){
    track_diff->handle = (void *)td;
    track_diff->update = trackDiffTDUpdate;
    track_diff->get_state = trackDiffTDGetState;
}

/**
 * @brief  句柄绑定到 LTD 微分器
 * @param  track_diff 微分器句柄
 * @param  ltd        LTD实例
 * @retval void
 */
void TrackDiffBindLTD(TrackDiff *track_diff, LTD *ltd){
    track_diff->handle = (void *)ltd;
    track_diff->update = trackDiffLTDUpdate;
    track_diff->get_state = trackDiffLTDGetState;
}

/**
 * @brief  统一入口推进微分器状态
 * @param  track_diff 微分器句柄
 * @param  target     目标量
 * @retval void
 * @note   未绑定时直接返回，不改变任何状态
 */
void TrackDiffUpdate(TrackDiff *track_diff, float target){
    if(track_diff->update == NULL || track_diff->handle == NULL){
        return;
    }
    track_diff->update(track_diff->handle, target);
}

/**
 * @brief  统一入口读取微分器状态
 * @param  track_diff 微分器句柄
 * @param  x1         一阶跟踪量输出
 * @param  x2         二阶跟踪量输出
 * @retval void
 * @note   未绑定时输出置零，保证调用方拿到确定值
 */
void TrackDiffGetState(const TrackDiff *track_diff, float *x1, float *x2){
    if(track_diff->get_state == NULL || track_diff->handle == NULL){
        *x1 = 0.0f;
        *x2 = 0.0f;
        return;
    }
    track_diff->get_state(track_diff->handle, x1, x2);
}

/*--------------------------------------------------- ESO 扩张状态观测器 ---------------------------------------------------*/

/**
 * @brief  ESO扩张状态观测器初始化
 * @param  eso       ESO结构体
 * @param  h         积分步长
 * @param  b         x_dot = Ax + Bu 中控制矩阵u关于一阶量的因子，取0则取消控制量修正，
 *                   将电机控制量囊括在外部扰动中
 * @param  b_01      z1_dot = h * (z2 - b_01 * error)
 * @param  b_02      z2_dot = h * (z3 - b_02 * Fal(error, 0.5, delta) + b*u)
 * @param  b_03      z3_dot = -h * (b_03 * Fal(error, 0.25, delta))
 * @param  z3_limit  扰动观测量限幅
 * @param  min       若一阶信号为周期信号给周期限幅下限，否则给0
 * @param  max       若一阶信号为周期信号给周期限幅上限，否则给0
 * @retval void
 * @note   一般取 b_01 = 1/h，b_02 = 1/(3*h*h)，b_03 = 1/(20*h*h*h)；
 *         Fal 线性区倍率默认取 1 倍步长，z1/z2 边界限幅默认关闭，
 *         逐轴差异分别由 ESOSetFalTuning() 与 ESOSetStateLimit() 配置
 */
void ESOInitialize(ESO *eso, float h, float b, float b_01, float b_02, float b_03, float z3_limit, float min, float max){
    eso->h = h;
    eso->b = b;
    eso->beta_01 = b_01;
    eso->beta_02 = b_02;
    eso->beta_03 = b_03;
    eso->z3_limit = z3_limit;
    eso->max = max;
    eso->min = min;

    /* 逐轴整定项默认值：与未参数化前的 pitch 观测器保持一致 */
    eso->fal_delta_gain_z2 = ESO_FAL_DELTA_GAIN_UNIT;
    eso->fal_delta_gain_z3 = ESO_FAL_DELTA_GAIN_UNIT;
    eso->state_limit_enable = ESO_STATE_LIMIT_DISABLE;
}

/**
 * @brief  ESO非线性区宽度整定接口
 * @param  eso                 ESO结构体
 * @param  fal_delta_gain_z2   z2支路Fal线性区宽度相对步长的倍率
 * @param  fal_delta_gain_z3   z3支路Fal线性区宽度相对步长的倍率
 * @retval void
 * @note   倍率越大线性区越宽，观测越平滑但对小误差的收敛越慢；
 *         yaw 轴整定为 10 倍步长，pitch 轴保持 1 倍步长
 */
void ESOSetFalTuning(ESO *eso, float fal_delta_gain_z2, float fal_delta_gain_z3){
    eso->fal_delta_gain_z2 = fal_delta_gain_z2;
    eso->fal_delta_gain_z3 = fal_delta_gain_z3;
}

/**
 * @brief  ESO观测量边界限幅设置，并使能 z1/z2 限幅
 * @param  eso    ESO结构体
 * @param  z1_min 一阶观测量下限
 * @param  z1_max 一阶观测量上限
 * @param  z2_min 二阶观测量下限
 * @param  z2_max 二阶观测量上限
 * @retval void
 * @note   仅在四个边界都已按物理量程给定时调用；未调用的轴保持限幅关闭，
 *         避免边界为0时把观测量直接压成0
 */
void ESOSetStateLimit(ESO *eso, float z1_min, float z1_max, float z2_min, float z2_max){
    eso->z1_min = z1_min;
    eso->z1_max = z1_max;
    eso->z2_min = z2_min;
    eso->z2_max = z2_max;
    eso->state_limit_enable = ESO_STATE_LIMIT_ENABLE;
}

/**
 * @brief  ESO扩张状态观测器更新
 * @param  eso          ESO结构体
 * @param  feedback     受控目标一阶返回观测值
 * @param  control_val  给予控制目标的先验控制量
 * @retval void
 * @note   没有控制输入时eso观测会乱瞟，故对 z3 做限幅并按需对 z1/z2 做边界限幅
 */
void ESOUpdate(ESO *eso, float feedback, float control_val){
    float e = eso->z1 - feedback;
    float fal_delta_z2 = eso->fal_delta_gain_z2 * eso->h;
    float fal_delta_z3 = eso->fal_delta_gain_z3 * eso->h;

    if(eso->min != eso->max){
        e = AngleLimit(e, eso->min, eso->max);
    }

    eso->z1 += eso->h * (eso->z2 - eso->beta_01 * e);
    if(eso->b != 0){
        eso->z2 += eso->h * (eso->z3 - eso->beta_02 * Fal(e, ESO_FAL_ALPHA_Z2, fal_delta_z2) + eso->b * control_val);
    } else {
        eso->z2 += eso->h * (eso->z3 - eso->beta_02 * Fal(e, ESO_FAL_ALPHA_Z2, fal_delta_z2));
    }
    eso->z3 -= eso->h * (eso->beta_03 * Fal(e, ESO_FAL_ALPHA_Z3, fal_delta_z3));
    eso->z3 = AbsLimiter(eso->z3, eso->z3_limit);

    if(eso->min != eso->max){
        eso->z1 = AngleLimit(eso->z1, eso->min, eso->max);
    }
    if(eso->state_limit_enable == ESO_STATE_LIMIT_ENABLE){
        eso->z2 = DoubleEdgeLimiter(eso->z2, eso->z2_min, eso->z2_max);
        eso->z1 = DoubleEdgeLimiter(eso->z1, eso->z1_min, eso->z1_max);
    }
}

/**
 * @brief  ESO状态复位：z1对齐当前值，z2/z3清零
 * @param  eso ESO结构体
 * @param  z1  复位目标值（通常为当前反馈角）
 * @retval void
 */
void ESO_Reset(ESO *eso, float z1){
    eso->z1 = z1;
    eso->z2 = 0.0f;
    eso->z3 = 0.0f;
}

/*---------------------------------------------------- ESF 线性组合器 ----------------------------------------------------*/

/**
 * @brief  线性组合器初始化
 * @param  esf           ESF结构体
 * @param  k_0           误差积分放大系数
 * @param  k_1           误差放大系数
 * @param  k_2           误差微分放大系数
 * @param  e_0_max       误差积分上限
 * @param  output_limit  输出限幅
 * @retval void
 */
void LESFInitialize(ESF *esf, float k_0, float k_1, float k_2, float e_0_max, float output_limit){
    esf->k_0 = k_0;
    esf->k_1 = k_1;
    esf->k_2 = k_2;
    esf->e_0_max = e_0_max;
    esf->output_limit = output_limit;
}

/**
 * @brief  线性组合器计算更新
 * @param  esf ESF结构体
 * @param  e_1 一阶误差输入
 * @param  e_2 二阶误差输入
 * @retval float 组合输出
 */
float LESFUpdate(ESF *esf, float e_1, float e_2){
    esf->e_0 += e_1; // 相当于I
    esf->e_1 = e_1;
    esf->e_2 = e_2;
    esf->e_0 = AbsLimiter(esf->e_0, esf->e_0_max);
    esf->output = esf->e_0 * esf->k_0 + e_1 * esf->k_1 + e_2 * esf->k_2;
    esf->output = AbsLimiter(esf->output, esf->output_limit);
    return esf->output;
}

/*------------------------------------------------- 二阶误差速度来源（函数指针） -------------------------------------------------*/

/**
 * @brief  取观测器估计速度作为二阶误差参考
 * @param  adrc ADRC结构体
 * @retval float ESO 观测速度 z2
 * @note   yaw 轴采用该来源
 */
float ADRCVelocityFromObserver(const ADRC *adrc){
    return adrc->eso.z2;
}

/**
 * @brief  取云台pitch实测角速度作为二阶误差参考
 * @param  adrc ADRC结构体
 * @retval float pitch 实测角速度(dps)
 * @note   pitch 轴采用该来源，实测替换观测能明显改善跟踪表现
 */
float ADRCVelocityFromPitchEstimate(const ADRC *adrc){
    (void)adrc;
    return _gimbalControl->GimbalEstimate.pitch_angular_velocity_dps;
}

/*------------------------------------------------------- ADRC 本体 -------------------------------------------------------*/

/**
 * @brief  LADRC结构体初始化
 * @param  adrc           adrc结构体
 * @param  td_init_val    td环节参数 td_init_val[3] = {r, h0, N}
 * @param  lesf_init_val  esf环节参数 lesf_init_val[5] = {k_0, k_1, k_2, e_0_max, output_limit}
 * @param  eso_init_val   eso环节参数 eso_init_val[6] = {h, b, beta_01, beta_02, beta_03, z3_limit}
 * @param  min            若处理的信号为周期函数，min为周期下限
 * @param  max            若处理的信号为周期函数，max为周期上限，周期在adrc不同环节内统一，
 *                        若为非周期信号，请输入任意 min = max
 * @retval void
 * @note   严禁修改结构体内变量顺序，给予参数请严格按照注释给定；
 *         默认绑定内部 td 作为微分器、eso.z2 作为二阶误差速度来源、z3 全量补偿
 */
void LADRCInitialize(ADRC *adrc, float *td_init_val, float *lesf_init_val, float *eso_init_val, float min, float max){
    TDInitialize(&adrc->td, td_init_val[0], td_init_val[1], td_init_val[2], min, max);
    LESFInitialize(&adrc->esf, lesf_init_val[0], lesf_init_val[1], lesf_init_val[2], lesf_init_val[3], lesf_init_val[4]);
    ESOInitialize(&adrc->eso, eso_init_val[0], eso_init_val[1], eso_init_val[2], eso_init_val[3], eso_init_val[4], eso_init_val[5], min, max);
    adrc->limit_max = max;
    adrc->limit_min = min;

    /* 默认接线：TD 微分器 + 观测速度 + 全量扰动补偿 */
    TrackDiffBindTD(&adrc->track_diff, &adrc->td);
    adrc->velocity_ref = ADRCVelocityFromObserver;
    adrc->z3_gain = ADRC_Z3_GAIN_FULL;
}

/**
 * @brief  把ADRC的微分器切换为外部LTD实例
 * @param  adrc adrc结构体
 * @param  ltd  外部LTD实例
 * @retval void
 * @note   LTD 比内部 TD 高一阶且步长由 DWT 实测给出，pitch 轴使用该微分器
 */
void ADRCBindTrackDiffLTD(ADRC *adrc, LTD *ltd){
    TrackDiffBindLTD(&adrc->track_diff, ltd);
}

/**
 * @brief  绑定二阶误差速度来源
 * @param  adrc          adrc结构体
 * @param  velocity_ref  速度来源回调，传 NULL 表示回退到观测速度 z2
 * @retval void
 */
void ADRCBindVelocityRef(ADRC *adrc, VelocityRefFunc velocity_ref){
    adrc->velocity_ref = (velocity_ref != NULL) ? velocity_ref : ADRCVelocityFromObserver;
}

/**
 * @brief  自抗扰计算内核
 * @param  adrc               自抗扰计算所需结构体
 * @param  target             控制目标期望值
 * @param  feedback           控制目标反馈值
 * @param  velocity_override  调用级速度来源覆盖，传 NULL 表示使用绑定的回调
 * @retval void
 * @note   计算步骤：
 *         1. 微分器推进，给出期望位置x1与期望速度x2；
 *         2. LESF 组合一阶误差与二阶误差，得到未补偿控制量u_0；
 *         3. 按 z3_gain 权重扣除观测扰动并除以控制增益b，得到最终控制量u；
 *         4. 观测器吃进反馈与u，更新位置、速度与扰动估计
 */
static void adrcUpdateCore(ADRC *adrc, float target, float feedback, const float *velocity_override){
    float x1 = 0.0f;
    float x2 = 0.0f;
    float velocity_ref;

    TrackDiffUpdate(&adrc->track_diff, target);
    TrackDiffGetState(&adrc->track_diff, &x1, &x2);

    /* 速度来源优先级：调用级覆盖 > 绑定的回调 > 观测速度z2 */
    if(velocity_override != NULL){
        velocity_ref = *velocity_override;
    } else if(adrc->velocity_ref != NULL){
        velocity_ref = adrc->velocity_ref(adrc);
    } else {
        velocity_ref = adrc->eso.z2;
    }

    adrc->u_0 = LESFUpdate(&adrc->esf,
                           AngleLimit(x1 - adrc->eso.z1, adrc->limit_min, adrc->limit_max),
                           x2 - velocity_ref);

    /* 最终控制量 = LESF输出 - 观测扰动补偿，再除以控制增益 */
    if(adrc->eso.b == 0){
        adrc->u = adrc->u_0;
    } else {
        adrc->u = (adrc->u_0 - adrc->z3_gain * adrc->eso.z3) / (adrc->eso.b * 1.0f);
    }

    ESOUpdate(&adrc->eso, feedback, adrc->u);
}

/**
 * @brief  自抗扰统一计算入口
 * @param  adrc      自抗扰计算所需结构体
 * @param  target    控制目标期望值
 * @param  feedback  控制目标反馈值
 * @retval void
 * @note   微分器种类与二阶误差速度来源由绑定的函数指针决定，因此 TD/LTD、
 *         观测速度/实测速度的各种组合都收口到本入口，上层无需再区分调用哪个函数
 */
void ADRCUpdate(ADRC *adrc, float target, float feedback){
    adrcUpdateCore(adrc, target, feedback, NULL);
}

/*---------------------------------------------------- 兼容调用入口 ----------------------------------------------------*/

/**
 * @brief  线性自抗扰计算（TD微分器 + 观测速度）
 * @param  adrc      自抗扰计算所需结构体
 * @param  target    控制目标期望值
 * @param  feedback  控制目标反馈值
 * @retval void
 * @note   等价于绑定内部TD与观测速度后调用 ADRCUpdate()，保留供既有调用点使用
 */
void LADRCUpdate(ADRC *adrc, float target, float feedback){
    ADRCUpdate(adrc, target, feedback);
}

/**
 * @brief  yaw轴线性自抗扰计算（TD微分器 + 观测速度 + 10倍步长非线性区）
 * @param  adrc      自抗扰计算所需结构体
 * @param  target    控制目标期望值(rad)
 * @param  feedback  控制目标反馈值(rad)
 * @retval void
 * @note   yaw 观测器沿用 47c7530 的整定：z2支路Fal线性区取10倍步长、z3支路取1倍步长，
 *         且不做 z1/z2 边界限幅（yaw 未给定边界，限幅会把观测量压成0）
 */
void YawLADRCUpdate(ADRC *adrc, float target, float feedback){
    ADRCUpdate(adrc, target, feedback);
}

/**
 * @brief  线性自抗扰计算 V2 —— 二阶误差使用实测速度并按权重补偿扰动
 * @param  adrc             自抗扰计算所需结构体
 * @param  target           控制目标期望值(rad)
 * @param  feedback         控制目标反馈值(rad)
 * @param  actual_velocity  实际测量角速度(rad/s)，替代 eso.z2 参与 LESF 组合
 * @param  z3_gain          扰动补偿权重(0~1)，乘在z3上：u = (u0 - z3_gain*z3) / b
 * @retval void
 */
void LADRCUpdateV2(ADRC *adrc, float target, float feedback, float actual_velocity, float z3_gain){
    TrackDiffBindTD(&adrc->track_diff, &adrc->td);
    adrc->z3_gain = z3_gain;
    adrcUpdateCore(adrc, target, feedback, &actual_velocity);
    adrc->z3_gain = ADRC_Z3_GAIN_FULL;
}

/**
 * @brief  pitch轴自抗扰计算（外部LTD微分器 + pitch实测角速度）
 * @param  adrc      自抗扰计算所需结构体
 * @param  ltd       外部LTD微分器，给出期望目标位置与速度
 * @param  target    控制目标期望值
 * @param  feedback  控制目标反馈值
 * @retval void
 * @note   二阶误差用实测角速度替换观测速度z2，实测效果明显优于纯观测
 */
void LTDADRCUpdate(ADRC *adrc, LTD *ltd, float target, float feedback){
    /* 兼容旧调用：微分器实例由参数给出，故此处仍做一次绑定校正 */
    if(adrc->track_diff.handle != (void *)ltd){
        ADRCBindTrackDiffLTD(adrc, ltd);
    }
    ADRCUpdate(adrc, target, feedback);
}

/**
 * @file worldGimbal.c
 * @brief 世界系云台控制实现：虚拟目标 f_des_B + 阻尼最小二乘IK反解
 * @note  坐标系：B系 x=前 y=右 z=下（右手系）
 */

#include "worldGimbal.h"
#include "gimbalControl.h"              /* GimbalControl, PITCH_OFFSET_MACHENICAL_ANGLE, AngleLimit(via algorism.h) */
#include "general_config_label.h"       /* LK_FULL_CIRCLE_MECHENICAL_ANGLE */
#include "peripheral_receive_task.h"    /* DJIGMotorRec */
#include <math.h>
#include <string.h>

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

/* ---- 全局实例 ---- */
WorldGimbal worldGimbal = {0};
const WorldGimbal* _worldGimbal = &worldGimbal;

/* 外部引用：底盘IMU（485下板传输） */
extern volatile float g_b2b_body_pitch_d;
extern volatile float g_b2b_body_roll_d;
extern volatile float g_b2b_body_yaw_d;

/* 外部引用：yaw DM 编码器（上板本地 CAN3，替代原下板485转发） */
extern DMJ4310MotorRec DMyawMotorRec;
extern float yaw_dm_forward_offset_rad;

/* 外部引用：pitch电机编码器（上板本地CAN，用于FK/IK） */
extern DJIGMotorRec pitchMotorRec;

/* 外部引用：云台控制结构体（写入IK反解结果） */
extern GimbalControl gimbalControl;

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
/* yaw角度：本地 DM 编码器 (CAN3)，直接读取不再依赖下板485转发 */
static inline float WG_GetCurrentYawDeg(void)
{
    return AngleLimit((DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f, -180.0f, 180.0f);
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
void WG_WorldAnglesToFdesB(float world_yaw_deg, float world_pitch_deg,
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
    wg->WorldGimbalEstimate.chassis_roll_deg  = g_b2b_body_roll_d;
    wg->WorldGimbalEstimate.chassis_pitch_deg = g_b2b_body_pitch_d;
    wg->WorldGimbalEstimate.chassis_yaw_deg   = g_b2b_body_yaw_d;

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

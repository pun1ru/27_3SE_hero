/**
 * @file    Board2Board.c
 * @brief   双板 CAN 通信驱动实现（上板端 — 接收）
 * @note    CAN 帧解析后写入目标由参数指定，调用处一目了然。
 *          写入目标与 485 路径一致，消费端无需改动。
 */

#include "Board2Board.h"
#include "general_task_include.h"
#include <math.h>

/* ---- 485 原有全局变量 ---- */
extern NormRemoteCmd       normRemoteCmd;
extern volatile float      g_b2b_yaw_cmd_d;
extern volatile float      g_b2b_pitch_cmd_d;
extern volatile float      g_b2b_stir_toq;
extern volatile float      g_b2b_stir_vel;
extern volatile uint8_t    g_b2b_yaw_cmd_valid;
extern volatile uint8_t    g_b2b_shoot_flag;
extern volatile float      g_b2b_bullet_speed;
extern volatile float      g_b2b_body_pitch_d;
extern volatile float      g_b2b_body_roll_d;
extern volatile float      g_b2b_body_yaw_d;
extern volatile uint16_t   g_b2b_current_hp;
extern volatile uint8_t    g_b2b_hp_zero_flag;
extern EventGroupHandle_t  remoteRecEventGroup;

/* ---- 模式缓存 ---- */
static uint8_t b2b_switch_r = NORM_RC_SW_MID;

/* ---- B2B 下行（下板→上板）心跳 ---- */
#define B2B_DOWN_ALIVE_THRESHOLD  20U   /* 20周期×~2ms ≈ 40ms超时 */
volatile uint32_t g_b2b_down_alive_ctr = 0;  /* ISR置位，task递减 */
volatile uint8_t  g_b2b_down_valid = 0;      /* 0=下行丢失 */

/* ======================================================================
 * 内部辅助
 * ====================================================================== */

static inline int16_t b2bReadI16BE(const uint8_t* src)
{
    return (int16_t)((src[0] << 8) | src[1]);
}

static inline uint16_t b2bReadU16BE(const uint8_t* src)
{
    return (uint16_t)((src[0] << 8) | src[1]);
}

static inline void b2bWriteI16BE(uint8_t* dst, int16_t val)
{
    dst[0] = (uint8_t)(val >> 8);
    dst[1] = (uint8_t)(val);
}

static inline float b2bReadF32(const uint8_t* src)
{
    float val;
    memcpy(&val, src, sizeof(val));
    return val;
}

static inline void b2bWriteF32(uint8_t* dst, float val)
{
    memcpy(dst, &val, sizeof(val));
}

/* ======================================================================
 * 帧解析 — 写入目标由参数指针指定
 * ====================================================================== */

/** 帧格式
 * @brief   解析 0x220 机体姿态
 */
static void b2bParseBodyState(uint8_t* data,
                              float* body_pitch,
                              float* body_roll,
                              float* body_yaw)
{
    *body_pitch  = (float)b2bReadI16BE(data + 2) / 100.0f;
    *body_roll   = (float)b2bReadI16BE(data + 0) / 100.0f;
    *body_yaw    = (float)b2bReadI16BE(data + 4) / 100.0f;
}

/** 帧格式
 * @brief   解析 0x221 云台最终目标（yaw float + pitch float）
 */
static void b2bParseGimbalInput(uint8_t* data, float* yaw_cmd_deg, float* pitch_cmd_deg)
{
    *yaw_cmd_deg   = b2bReadF32(data + 0);
    *pitch_cmd_deg = b2bReadF32(data + 4);
}

static void b2bParseShootState(uint8_t* data, uint8_t* shoot_flag, float* bullet_speed)
{
    *shoot_flag  = data[0];
    *bullet_speed = b2bReadF32(data + 1);
}

/** 帧格式
 * @brief   解析 0x223 拨盘数据（stir_toq + stir_vel）
 * @note    拨盘在 hero_down，力矩 (Nm) 和速度 (rad/s) 经 B2B 转发，×100。
 */
static void b2bParseStir(uint8_t* data, float* stir_toq, float* stir_vel)
{
    *stir_toq = (float)b2bReadI16BE(data + 0) / 100.0f;
    *stir_vel = (float)b2bReadI16BE(data + 2) / 100.0f;
}

/**
 * @brief   解析 0x111 键位 + 开关 + HP → NormRemoteCmd + hp_out
 */
static void b2bParseKeysSwitch(uint8_t* data,
                               NormRemoteCmd* dst,
                               uint16_t* hp_out)
{
    uint8_t  sb    = data[2];
    uint16_t pckey = b2bReadU16BE(data + 0);
    uint8_t  sw_r  = (sb >> 0) & 0x03U;
    uint8_t  sw_l  = (sb >> 2) & 0x03U;
    uint16_t hp    = b2bReadU16BE(data + 6);

    dst->remote_source = DT7;  /* B2B CAN 替代 RC_UART */
    memcpy(&dst->PCKeyBoard, &pckey, sizeof(pckey));
    dst->Switch.switch_R1    = sw_r;
    dst->Switch.switch_L1    = sw_l;
    dst->PCMouse.mouse_left  = (sb >> 4) & 0x01U;
    dst->PCMouse.mouse_right = (sb >> 5) & 0x01U;
    dst->RelativeCH.ch4      = (float)b2bReadI16BE(data + 4) / 1000.0f;

    *hp_out = hp;
    if (hp == 0)
        g_b2b_hp_zero_flag = 1;

    b2b_switch_r = sw_r;
}

/* ======================================================================
 * API
 * ====================================================================== */

void B2BInit(void)
{
    b2b_switch_r = NORM_RC_SW_MID;

    /* 替代 RemoteRecTask：B2B CAN 直写 normRemoteCmd，不需要 UART 事件唤醒 */
    if (remoteRecEventGroup == NULL)
        remoteRecEventGroup = xEventGroupCreate();
}

uint8_t B2BCanRxHandler(uint16_t can_id, uint8_t* data)
{
    switch (can_id)
    {
        case B2B_DOWN_BODY_STATE:
            /* 机体姿态 */
            b2bParseBodyState(data,
                              (float*)&g_b2b_body_pitch_d,
                              (float*)&g_b2b_body_roll_d,
                              (float*)&g_b2b_body_yaw_d);
            g_b2b_down_alive_ctr = B2B_DOWN_ALIVE_THRESHOLD;
            g_b2b_down_valid = 1;
            return 1U;

        case B2B_DOWN_GIMBAL_INPUT:
            /* yaw_cmd + pitch_cmd 一起发来 */
        {
            float yaw_cmd;
            float pitch_cmd;
            b2bParseGimbalInput(data, &yaw_cmd, &pitch_cmd);
            if (!isfinite(yaw_cmd) || !isfinite(pitch_cmd)
                || fabsf(yaw_cmd) > 180.0f || fabsf(pitch_cmd) > 180.0f)
            {
                g_b2b_yaw_cmd_valid = 0;
                return 1U;
            }
            g_b2b_yaw_cmd_d = yaw_cmd;
            g_b2b_pitch_cmd_d = pitch_cmd;
            g_b2b_yaw_cmd_valid = 1;
            g_b2b_down_alive_ctr = B2B_DOWN_ALIVE_THRESHOLD;
            g_b2b_down_valid = 1;
            return 1U;
        }

        case B2B_DOWN_SHOOT_STATE:
        {
            uint8_t shoot_flag;
            float bullet_speed;
            b2bParseShootState(data, &shoot_flag, &bullet_speed);
            if (!isfinite(bullet_speed))
                return 1U;
            g_b2b_shoot_flag = shoot_flag;
            g_b2b_bullet_speed = bullet_speed;
            return 1U;
        }

        case B2B_DOWN_KEYS_SWITCH:
            b2bParseKeysSwitch(data, &normRemoteCmd, (uint16_t*)&g_b2b_current_hp);
            return 1U;

        case B2B_DOWN_STIR:
            b2bParseStir(data, (float*)&g_b2b_stir_toq, (float*)&g_b2b_stir_vel);
            g_b2b_down_alive_ctr = B2B_DOWN_ALIVE_THRESHOLD;
            g_b2b_down_valid = 1;
            return 1U;

        default:
            return 0U;
    }
}

/* ======================================================================
 * B2B 下行心跳检测
 * ====================================================================== */

/**
 * @brief   B2B 下板帧超时检测（ControlTask 每周期调用）
 * @note    递减 alive 计数器，归零时标记 g_b2b_down_valid = 0
 *          B2B 下行 500Hz（2ms/帧），阈值 20→~40ms 超时
 *          B2BCanRxHandler 每收到一帧就重置计数器
 */
void B2B_DownAliveCheck(void)
{
    if (g_b2b_down_alive_ctr > 0U) {
        g_b2b_down_alive_ctr--;
    }
    if (g_b2b_down_alive_ctr == 0U) {
        g_b2b_down_valid = 0;
    }
}

/* ======================================================================
 * 上板→下板 发送
 * ====================================================================== */

uint8_t B2BSendGimbalPose(float yaw_d, float pitch_d, float yaw_dps, float pitch_dps)
{
    uint8_t data[8];
    uint8_t pose_status;

    b2bWriteF32(data + 0, yaw_d);
    b2bWriteF32(data + 4, pitch_d);
    pose_status = fdcanx_send_data(&B2B_CAN, B2B_UP_GIMBAL_POSE, data, 8);

    b2bWriteF32(data + 0, yaw_dps);
    b2bWriteF32(data + 4, pitch_dps);
    return (uint8_t)(pose_status | fdcanx_send_data(&B2B_CAN, B2B_UP_GIMBAL_VELOCITY, data, 8));
}

uint8_t B2BSendGimbalTarget(float target_yaw, float target_pitch)
{
    uint8_t data[8];

    b2bWriteF32(data + 0, target_yaw);
    b2bWriteF32(data + 4, target_pitch);

    return fdcanx_send_data(&B2B_CAN, B2B_UP_GIMBAL_TARGET, data, 8);
}

uint8_t B2BSendFricRPM(const int16_t rpm[6])
{
    uint8_t data[8];

    /* 0x202: rpm[0..3] */
    b2bWriteI16BE(data + 0, rpm[0]);
    b2bWriteI16BE(data + 2, rpm[1]);
    b2bWriteI16BE(data + 4, rpm[2]);
    b2bWriteI16BE(data + 6, rpm[3]);
    fdcanx_send_data(&B2B_CAN, B2B_UP_FRIC_RPM_A, data, 8);

    /* 0x203: rpm[4..5] */
    b2bWriteI16BE(data + 0, rpm[4]);
    b2bWriteI16BE(data + 2, rpm[5]);
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    return fdcanx_send_data(&B2B_CAN, B2B_UP_FRIC_RPM_B, data, 8);
}

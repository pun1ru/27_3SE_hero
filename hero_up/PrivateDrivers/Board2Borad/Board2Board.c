/**
 * @file    Board2Board.c
 * @brief   双板 CAN 通信驱动实现（上板端 — 接收）
 * @note    CAN 帧解析后写入目标由参数指定，调用处一目了然。
 *          写入目标与 485 路径一致，消费端无需改动。
 */

#include "Board2Board.h"
#include "general_task_include.h"

/* ---- 485 原有全局变量 ---- */
extern NormRemoteCmd       normRemoteCmd;
extern volatile float      shoot485_yaw_rx_d;
extern volatile uint8_t    shoot485_yaw_rx_valid;
extern volatile float      servant485_pitch_d;
extern volatile float      servant485_roll_d;
extern volatile float      servant485_yaw_d;
extern volatile uint16_t   servant485_current_hp;
extern volatile uint8_t    servant485_hp_zero_flag;
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

/* ======================================================================
 * 帧解析 — 写入目标由参数指针指定
 * ====================================================================== */

/**
 * @brief   解析 0x100 机体姿态 + yaw 编码器
 */
static void b2bParseBodyState(uint8_t* data,
                              float* yaw_enc_deg,
                              float* body_pitch,
                              float* body_roll,
                              float* body_yaw)
{
    *yaw_enc_deg = (float)b2bReadI16BE(data + 6) / 100.0f;
    *body_pitch  = (float)b2bReadI16BE(data + 2) / 100.0f;
    *body_roll   = (float)b2bReadI16BE(data + 0) / 100.0f;
    *body_yaw    = (float)b2bReadI16BE(data + 4) / 100.0f;
}

/**
 * @brief   解析 0x221 云台 pitch 控制 → NormRemoteCmd
 * @note    上板只有 pitch 电机，帧内不传 yaw。
 *          pitch_cmd 用于 RC 模式（ch3 摇杆），mouse_speed_y 用于 PC 模式（鼠标）。
 */
static void b2bParseGimbalInput(uint8_t* data, NormRemoteCmd* dst)
{
    float pitch_cmd     = (float)b2bReadI16BE(data + 0) / 1000.0f;
    dst->PCMouse.mouse_speed_y = b2bReadI16BE(data + 2);

    dst->RelativeCH.ch0 = (float)b2bReadI16BE(data + 4) / 1000.0f;
    dst->RelativeCH.ch1 = (float)b2bReadI16BE(data + 6) / 1000.0f;
    dst->RelativeCH.ch3 = pitch_cmd;
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
        servant485_hp_zero_flag = 1;

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
            b2bParseBodyState(data,
                              (float*)&shoot485_yaw_rx_d,
                              (float*)&servant485_pitch_d,
                              (float*)&servant485_roll_d,
                              (float*)&servant485_yaw_d);
            shoot485_yaw_rx_valid = 1;
            g_b2b_down_alive_ctr = B2B_DOWN_ALIVE_THRESHOLD;
            g_b2b_down_valid = 1;
            return 1U;

        case B2B_DOWN_GIMBAL_INPUT:
            b2bParseGimbalInput(data, &normRemoteCmd);
            g_b2b_down_alive_ctr = B2B_DOWN_ALIVE_THRESHOLD;
            g_b2b_down_valid = 1;
            return 1U;

        case B2B_DOWN_KEYS_SWITCH:
            b2bParseKeysSwitch(data, &normRemoteCmd, (uint16_t*)&servant485_current_hp);
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

    b2bWriteI16BE(data + 0, (int16_t)(yaw_d    * 100.0f));
    b2bWriteI16BE(data + 2, (int16_t)(pitch_d  * 100.0f));
    b2bWriteI16BE(data + 4, (int16_t)(yaw_dps  * 100.0f));
    b2bWriteI16BE(data + 6, (int16_t)(pitch_dps * 100.0f));

    return fdcanx_send_data(&B2B_CAN, B2B_UP_GIMBAL_POSE, data, 8);
}

uint8_t B2BSendGimbalTarget(float target_yaw, float target_pitch)
{
    uint8_t data[8];

    b2bWriteI16BE(data + 0, (int16_t)(target_yaw   * 100.0f));
    b2bWriteI16BE(data + 2, (int16_t)(target_pitch * 100.0f));
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

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

/**
 * @file    Board2Board.c
 * @brief   双板 CAN 通信驱动实现（下板端 — 发送）
 * @note    仿照 serialleg 模式：发送端完成模式选择 + 语义打包，上板零解析消费。
 *          按频率分帧：
 *            B2BSendGimbalInput() → 500Hz，云台控制（鼠标或摇杆）
 *            B2BSendKeysSwitch()  →  50Hz，键位 + 开关
 *            B2BSendBodyState()   → 按需，  机体姿态 + yaw 编码器
 */

#include "Board2Board.h"
#include "CAN_driver.h"
#include "general_config_label.h"
#include "judge_receive.h"
#include "peripheral_receive_task.h"
#include <string.h>

/* ---- 遥控器归一化指令（只读） ---- */
extern const NormRemoteCmd* _normRemoteCmd;

/* ======================================================================
 * 内部辅助 — int16 BE 编码
 * ====================================================================== */

static inline void b2bWriteI16BE(uint8_t* dst, int16_t val)
{
    dst[0] = (uint8_t)(val >> 8);
    dst[1] = (uint8_t)(val);
}

static inline void b2bWriteU16BE(uint8_t* dst, uint16_t val)
{
    dst[0] = (uint8_t)(val >> 8);
    dst[1] = (uint8_t)(val);
}

/* ======================================================================
 * API 实现
 * ====================================================================== */

void B2BInit(void)
{
    /* CAN 总线已由 HAL 层启动，滤波器在 can_filter_init() 中配置 */
}

/**
 * @brief   发送云台控制输入帧
 * @note    模式自适应：PC 模式发鼠标，遥控模式发摇杆。
 *          上板收到后直接用语义值，无需判断模式。
 */
uint8_t B2BSendGimbalInput(void)
{
    uint8_t data[8];
    int16_t pitch_cmd, yaw_cmd;

    /* ---- 模式自适应：PC 用鼠标，RC 用摇杆 ---- */
    if (_normRemoteCmd->Switch.switch_R1 == NORM_RC_SW_DOWN)
    {
        /* PC 模式：鼠标控制云台 */
        pitch_cmd = (int16_t)(-_normRemoteCmd->PCMouse.mouse_speed_y);
        yaw_cmd   = (int16_t)( _normRemoteCmd->PCMouse.mouse_speed_x);
    }
    else
    {
        /* 遥控模式（SW_R = MID/UP）：摇杆控制云台，ch2→yaw, ch3→pitch */
        pitch_cmd = (int16_t)(_normRemoteCmd->RelativeCH.ch3 * 1000.0f);
        yaw_cmd   = (int16_t)(_normRemoteCmd->RelativeCH.ch2 * 1000.0f);
    }

    int16_t ch0 = (int16_t)(_normRemoteCmd->RelativeCH.ch0 * 1000.0f);
    int16_t ch1 = (int16_t)(_normRemoteCmd->RelativeCH.ch1 * 1000.0f);

    b2bWriteI16BE(data + 0, pitch_cmd);
    b2bWriteI16BE(data + 2, yaw_cmd);
    b2bWriteI16BE(data + 4, ch0);
    b2bWriteI16BE(data + 6, ch1);

    return fdcanx_send_data(&B2B_CAN, B2B_DOWN_GIMBAL_INPUT, data, 8);
}

/**
 * @brief   发送键位 + 开关帧
 * @note    16 键位域直接从 NormRemoteCmd.PCKeyBoard 内存拷贝，零开销。
 *          开关字节打包 SW_R/SW_L + 鼠标按键。
 */
uint8_t B2BSendKeysSwitch(void)
{
    uint8_t data[8];
    uint16_t pckey;
    uint8_t  switch_byte;
    int16_t  ch4;

    /* 16 键位域 — 直接拷贝 __packed 结构体的 2 字节 */
    memcpy(&pckey, &_normRemoteCmd->PCKeyBoard, sizeof(pckey));

    /* 开关字节：SW_R(2) | SW_L(2) | press_L(1) | press_R(1) | reserved(2) */
    switch_byte  = (uint8_t)((_normRemoteCmd->Switch.switch_R1 & 0x03U) << 0);
    switch_byte |= (uint8_t)((_normRemoteCmd->Switch.switch_L1 & 0x03U) << 2);
    switch_byte |= (uint8_t)((_normRemoteCmd->PCMouse.mouse_left  ? 1U : 0U) << 4);
    switch_byte |= (uint8_t)((_normRemoteCmd->PCMouse.mouse_right ? 1U : 0U) << 5);

    ch4 = (int16_t)(_normRemoteCmd->RelativeCH.ch4 * 1000.0f);

    b2bWriteU16BE(data + 0, pckey);
    data[2] = switch_byte;
    data[3] = 0;
    b2bWriteI16BE(data + 4, ch4);
    b2bWriteU16BE(data + 6, ext_game_robot_status.current_HP);

    return fdcanx_send_data(&B2B_CAN, B2B_DOWN_KEYS_SWITCH, data, 8);
}

/**
 * @brief   发送机体姿态 + yaw 编码器帧
 */
uint8_t B2BSendBodyState(float roll_d, float pitch_d, float yaw_d, float yaw_enc_deg)
{
    uint8_t data[8];

    int16_t roll     = (int16_t)(roll_d       * 100.0f);
    int16_t pitch    = (int16_t)(pitch_d      * 100.0f);
    int16_t yaw      = (int16_t)(yaw_d        * 100.0f);
    int16_t yaw_enc  = (int16_t)(yaw_enc_deg  * 100.0f);

    b2bWriteI16BE(data + 0, roll);
    b2bWriteI16BE(data + 2, pitch);
    b2bWriteI16BE(data + 4, yaw);
    b2bWriteI16BE(data + 6, yaw_enc);

    return fdcanx_send_data(&B2B_CAN, B2B_DOWN_BODY_STATE, data, 8);
}

/* ======================================================================
 * 上板→下板 接收
 * ====================================================================== */

/* ---- 485 原有全局变量（解析后写入） ---- */
extern volatile float   gimbal_yaw_rx_d;
extern volatile float   gimbal_pitch_rx_d;
extern volatile float   gimbal_yaw_dps_rx;
extern volatile float   gimbal_pitch_dps_rx;
extern volatile float   gimbal_yaw_target_rx_d;
extern volatile float   gimbal_pitch_target_rx_d;
extern volatile int16_t gimbal_fric_rpm_rx_arr[6];
extern volatile uint8_t gimbal_yaw_rx_valid;

/* ---- int16 BE 解码 ---- */
static inline int16_t b2bReadI16BE(const uint8_t* src)
{
    return (int16_t)((src[0] << 8) | src[1]);
}

/**
 * @brief   解析 0x200 云台姿态 → gimbal_*_rx
 */
static void b2bParseGimbalPose(uint8_t* data,
                                float* yaw_deg,
                                float* pitch_deg,
                                float* yaw_dps,
                                float* pitch_dps)
{
    *yaw_deg   = (float)b2bReadI16BE(data + 0) / 100.0f;
    *pitch_deg = (float)b2bReadI16BE(data + 2) / 100.0f;
    *yaw_dps   = (float)b2bReadI16BE(data + 4) / 10.0f;
    *pitch_dps = (float)b2bReadI16BE(data + 6) / 10.0f;
}

/**
 * @brief   解析 0x201 云台目标 → gimbal_*_target_rx
 */
static void b2bParseGimbalTarget(uint8_t* data,
                                  float* target_yaw,
                                  float* target_pitch)
{
    *target_yaw   = (float)b2bReadI16BE(data + 0) / 100.0f;
    *target_pitch = (float)b2bReadI16BE(data + 2) / 100.0f;
}

/* ---- 接收 dispatcher ---- */
uint8_t B2BCanRxHandler(uint16_t can_id, uint8_t* data)
{
    switch (can_id)
    {
        case B2B_UP_GIMBAL_POSE:
            b2bParseGimbalPose(data,
                               (float*)&gimbal_yaw_rx_d,
                               (float*)&gimbal_pitch_rx_d,
                               (float*)&gimbal_yaw_dps_rx,
                               (float*)&gimbal_pitch_dps_rx);
            gimbal_yaw_rx_valid = 1;
            return 1U;

        case B2B_UP_GIMBAL_TARGET:
            b2bParseGimbalTarget(data,
                                 (float*)&gimbal_yaw_target_rx_d,
                                 (float*)&gimbal_pitch_target_rx_d);
            return 1U;

        case B2B_UP_FRIC_RPM_A:
            gimbal_fric_rpm_rx_arr[0] = b2bReadI16BE(data + 0);
            gimbal_fric_rpm_rx_arr[1] = b2bReadI16BE(data + 2);
            gimbal_fric_rpm_rx_arr[2] = b2bReadI16BE(data + 4);
            gimbal_fric_rpm_rx_arr[3] = b2bReadI16BE(data + 6);
            return 1U;

        case B2B_UP_FRIC_RPM_B:
            gimbal_fric_rpm_rx_arr[4] = b2bReadI16BE(data + 0);
            gimbal_fric_rpm_rx_arr[5] = b2bReadI16BE(data + 2);
            return 1U;

        default:
            return 0U;
    }
}

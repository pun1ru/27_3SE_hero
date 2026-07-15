/**
 * @file    Board2Board.c
 * @brief   双板 CAN 通信驱动实现（下板端 — 发送）
 * @note    仿照 serialleg 模式：发送端完成模式选择 + 语义打包，上板零解析消费。
 *          按频率分帧：
 *            B2BSendGimbalInput() → 500Hz，云台 pitch 控制（摇杆 + 鼠标）
 *            B2BSendKeysSwitch()  →  50Hz，键位 + 开关
 *            B2BSendBodyState()   → 按需，  机体姿态 + yaw 编码器
 */

#include "Board2Board.h"
#include "general_task_include.h"

/* ---- 遥控器归一化指令（只读） ---- */
extern const NormRemoteCmd* _normRemoteCmd;
extern const RobotState* _robotState;

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
 * @brief   发送云台 pitch 控制输入帧
 * @note    上板只有 pitch 电机，只需 pitch 控制量，yaw 在下板直接处理。
 *          RC 模式用 ch3（摇杆），PC 模式用 mouse_speed_y（鼠标）。
 *          上板收到后直接用语义值，无需判断模式。
 */
uint8_t B2BSendGimbalInput(void)
{
    uint8_t data[8];

    /* pitch_cmd: RC=ch3摇杆, mouse_speed_y: PC=鼠标Y增量 */
    int16_t pitch_cmd     = (int16_t)(_normRemoteCmd->RelativeCH.ch3 * 1000.0f);
    int16_t mouse_speed_y = _normRemoteCmd->PCMouse.mouse_speed_y;
    int16_t ch0 = (int16_t)(_normRemoteCmd->RelativeCH.ch0 * 1000.0f);
    int16_t ch1 = (int16_t)(_normRemoteCmd->RelativeCH.ch1 * 1000.0f);

    b2bWriteI16BE(data + 0, pitch_cmd);
    b2bWriteI16BE(data + 2, mouse_speed_y);
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
    *yaw_dps   = (float)b2bReadI16BE(data + 4) / 100.0f;
    *pitch_dps = (float)b2bReadI16BE(data + 6) / 100.0f;
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
uint32_t b2b_pose_rx_count = 0;  /* B2B云台姿态帧接收计数，在线watch判断丢帧 */
volatile uint32_t can3_rx_isr_cnt = 0;  /* CAN3 RxFifo1Callback 触发计数，>0表示ISR在运行 */
volatile uint16_t can3_last_rx_id = 0;   /* CAN3最后收到的帧ID，丢帧时确认实际收到什么 */
volatile uint32_t g_b2b_pose_alive_ctr = 0;  /* B2B心跳：ISR置位→GimbalPoseUpdate递减，>0认为在线 */

/**
 * @brief   B2B 云台姿态超时检测（由 GimbalPoseUpdate 每周期调用）
 * @note    递减心跳计数器，归零时清零 gimbal_yaw_rx_valid 触发安全回退
 *          B2B 500Hz 发送 → 2ms/帧，超时阈值 ~10ms (5帧丢失)
 */
void B2B_PoseAliveTick(void)
{
    static uint16_t lost_beep_timer = 0;

    if (g_b2b_pose_alive_ctr > 0) {
        g_b2b_pose_alive_ctr--;
    }
    if (g_b2b_pose_alive_ctr == 0) {
        gimbal_yaw_rx_valid = 0;
    }

    /* 心跳丢失 → 直接短促"嘀"(500ms周期/前50ms响)，不经过music_task */
    if (gimbal_yaw_rx_valid == 0) {
        lost_beep_timer++;
        if (lost_beep_timer >= 250U) {      /* 500ms = 250×2ms */
            lost_beep_timer = 0;
        } else if (lost_beep_timer < 25U) {  /* 前50ms = 25×2ms 响 */
            __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 500);
        } else if (lost_beep_timer == 25U) {
            __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0);
        }
    } else {
        if (lost_beep_timer != 0) {
            __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0);  /* 恢复时立即关 */
            lost_beep_timer = 0;
        }
    }
}

uint8_t B2BCanRxHandler(uint16_t can_id, uint8_t* data)
{
    switch (can_id)
    {
        case B2B_UP_GIMBAL_POSE:
            b2b_pose_rx_count++;
            g_b2b_pose_alive_ctr = 10U;  /* 10ms超时（≈5帧@500Hz），IMU 1kHz = 10次tick */
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

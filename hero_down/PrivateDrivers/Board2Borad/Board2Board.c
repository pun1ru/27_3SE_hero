/**
 * @file    Board2Board.c
 * @brief   双板 CAN 通信驱动实现（下板端 — 发送）
 * @note    仿照 serialleg 模式：发送端完成模式选择 + 语义打包，上板零解析消费。
 *          按频率分帧：
 *            B2BSendGimbalInput() → 100Hz，云台目标（yaw_cmd + pitch_cmd）
 *            B2BSendKeysSwitch()  → 100Hz，键位 + 开关
 *            B2BSendBodyState()   → 500Hz，机体姿态
 */

#include "Board2Board.h"
#include "general_task_include.h"
#include <math.h>

/* ---- 遥控器归一化指令（只读） ---- */
extern const NormRemoteCmd* _normRemoteCmd;
extern const RobotState* const _robotState;

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

static inline void b2bWriteF32(uint8_t* dst, float val)
{
    memcpy(dst, &val, sizeof(val));
}

/* ======================================================================
 * API 实现
 * ====================================================================== */

void B2BInit(void)
{
    /* CAN 总线已由 HAL 层启动，滤波器在 can_filter_init() 中配置 */
}

/**
 * @brief   发送云台最终目标（yaw float + pitch float）
 */
uint8_t B2BSendGimbalInput(float yaw_cmd_d, float pitch_cmd_d)
{
    uint8_t data[8];

    b2bWriteF32(data + 0, yaw_cmd_d);
    b2bWriteF32(data + 4, pitch_cmd_d);

    return fdcanx_send_data(&B2B_CAN, B2B_DOWN_GIMBAL_INPUT, data, 8);
}

uint8_t B2BSendShootState(void)
{
    uint8_t data[8] = {0};
    extern ext_shoot_data_t ext_shoot_data;

    data[0] = (uint8_t)((pDecisionAO->stir_mode != STIR_LOCK) ? 1U : 0U);
    b2bWriteF32(data + 1, ext_shoot_data.initial_speed);

    return fdcanx_send_data(&B2B_CAN, B2B_DOWN_SHOOT_STATE, data, 8);
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
 * @brief   发送机体姿态帧
 */
uint8_t B2BSendBodyState(float roll_d, float pitch_d, float yaw_d)
{
    uint8_t data[8] = {0};

    int16_t roll  = (int16_t)(roll_d  * 100.0f);
    int16_t pitch = (int16_t)(pitch_d * 100.0f);
    int16_t yaw   = (int16_t)(yaw_d   * 100.0f);

    b2bWriteI16BE(data + 0, roll);
    b2bWriteI16BE(data + 2, pitch);
    b2bWriteI16BE(data + 4, yaw);
    /* data[6..7] 保留置零 */

    return fdcanx_send_data(&B2B_CAN, B2B_DOWN_BODY_STATE, data, 8);
}

/* ======================================================================
 * B2BSendStir — 拨盘数据帧（500Hz）
 * ====================================================================== */

uint8_t B2BSendStir(void)
{
    uint8_t data[8] = {0};
    extern DMJ4310MotorRec stirMotorRec;

    int16_t stir_toq = (int16_t)(stirMotorRec.toq       * 100.0f);
    int16_t stir_vel = (int16_t)(stirMotorRec.vel_radps * 100.0f);

    b2bWriteI16BE(data + 0, stir_toq);
    b2bWriteI16BE(data + 2, stir_vel);
    /* data[4..7] 保留置零 */

    return fdcanx_send_data(&B2B_CAN, B2B_DOWN_STIR, data, 8);
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

static inline float b2bReadF32(const uint8_t* src)
{
    float val;
    memcpy(&val, src, sizeof(val));
    return val;
}

/**
 * @brief   解析 0x200 云台姿态 → gimbal_*_rx
 */
static void b2bParseGimbalPose(uint8_t* data,
                                float* yaw_deg,
                                float* pitch_deg)
{
    *yaw_deg   = b2bReadF32(data + 0);
    *pitch_deg = b2bReadF32(data + 4);
}

static void b2bParseGimbalVelocity(uint8_t* data,
                                    float* yaw_dps,
                                    float* pitch_dps)
{
    *yaw_dps   = b2bReadF32(data + 0);
    *pitch_dps = b2bReadF32(data + 4);
}

/**
 * @brief   解析 0x201 云台目标 → gimbal_*_target_rx
 */
static void b2bParseGimbalTarget(uint8_t* data,
                                  float* target_yaw,
                                  float* target_pitch)
{
    *target_yaw   = b2bReadF32(data + 0);
    *target_pitch = b2bReadF32(data + 4);
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
        {
            float yaw_deg;
            float pitch_deg;
            b2b_pose_rx_count++;
            b2bParseGimbalPose(data, &yaw_deg, &pitch_deg);
            if (!isfinite(yaw_deg) || !isfinite(pitch_deg)
                || fabsf(yaw_deg) > 180.0f || fabsf(pitch_deg) > 180.0f)
                return 1U;
            gimbal_yaw_rx_d = yaw_deg;
            gimbal_pitch_rx_d = pitch_deg;
            g_b2b_pose_alive_ctr = 30U;  /* 60ms超时（≈30帧@500Hz），容忍抖动不误报 */
            gimbal_yaw_rx_valid = 1;
            return 1U;
        }

        case B2B_UP_GIMBAL_VELOCITY:
        {
            float yaw_dps;
            float pitch_dps;
            b2bParseGimbalVelocity(data, &yaw_dps, &pitch_dps);
            if (!isfinite(yaw_dps) || !isfinite(pitch_dps))
                return 1U;
            gimbal_yaw_dps_rx = yaw_dps;
            gimbal_pitch_dps_rx = pitch_dps;
            return 1U;
        }

        case B2B_UP_GIMBAL_TARGET:
        {
            float target_yaw;
            float target_pitch;
            b2bParseGimbalTarget(data, &target_yaw, &target_pitch);
            if (!isfinite(target_yaw) || !isfinite(target_pitch)
                || fabsf(target_yaw) > 180.0f || fabsf(target_pitch) > 180.0f)
                return 1U;
            gimbal_yaw_target_rx_d = target_yaw;
            gimbal_pitch_target_rx_d = target_pitch;
            return 1U;
        }

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

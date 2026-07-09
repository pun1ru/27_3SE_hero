/**
 * @file    Board2Board.h
 * @brief   双板 CAN 通信驱动（下板端）
 * @note    仿照 serialleg 的简洁 CAN 帧设计：
 *          - 发送端完成模式选择 + 数据打包，接收端零解析直接消费语义值
 *          - 按频率拆分：高频帧发云台控制，低频帧发键位开关
 *          - 全部 8 字节单帧，CAN 硬件保证帧同步和 CRC
 */

#ifndef _BOARD2BOARD_H_
#define _BOARD2BOARD_H_

#include "fdcan.h"
#include <stdint.h>

/* ===================================================================
 * CAN ID 分配 — B2B 专用段 0x220-0x22F (hfdcan1, 双板直连)
 *
 * 不与 CAN1 现有 ID 冲突：
 *   CAN1 电机: 0x01-0x08(MIT)   0x141(LK pitch)
 *   滤波器:    RANGE [0x000,0x300] → 0x220-0x22F 全部放行
 * =================================================================== */
#define B2B_CAN                hfdcan1  /**< 双板通信用 CAN 总线（与上板 CAN1 直连） */

/* ── 下板→上板 ───────────────────────────── */
#define B2B_DOWN_BODY_STATE    0x220U  /**< 机体姿态 + yaw 编码器    500Hz(2ms) */
#define B2B_DOWN_GIMBAL_INPUT  0x221U  /**< 云台控制输入(模式自适应) 500Hz(2ms) */
#define B2B_DOWN_KEYS_SWITCH   0x222U  /**< 键位 + 开关 + HP         100Hz(10ms,同RC) */

/* ── 上板→下板 ───────────────────────────── */
#define B2B_UP_GIMBAL_POSE     0x228U  /**< 云台姿态（高频）                 */
#define B2B_UP_GIMBAL_TARGET   0x229U  /**< 云台目标（低频）                 */
#define B2B_UP_FRIC_RPM_A      0x22AU  /**< 摩擦轮转速 0..3                  */
#define B2B_UP_FRIC_RPM_B      0x22BU  /**< 摩擦轮转速 4..5                  */

/* ============================= API =================================== */

void B2BInit(void);

/* ---- 下板→上板 发送 ---- */

/**
 * @brief   发送云台控制输入帧（500Hz / 2ms，每 slot）
 * @note    自动根据 _normRemoteCmd->Switch.switch_R1 选择数据源：
 *          PC 模式（SW_R = DOWN）→ 鼠标 X/Y
 *          遥控模式             → 摇杆 ch2/ch3
 *          帧格式（8 字节）：
 *          [0..1] pitch_cmd int16  PC=-mouse_y, RC=ch3×1000
 *          [2..3] yaw_cmd   int16  PC=mouse_x,  RC=ch2×1000
 *          [4..5] ch0       int16  ×1000
 *          [6..7] ch1       int16  ×1000
 * @retval  0: 发送成功, 1: 发送失败
 */
uint8_t B2BSendGimbalInput(void);

/**
 * @brief   发送键位 + 开关帧（100Hz / 10ms，同 RC 接收周期，每 5 slot）
 * @note    帧格式（8 字节）：
 *          [0..1] PCKey       uint16  16 键位域
 *          [2]    switch_byte uint8   SW_R(2)|SW_L(2)|press_L|press_R|rsv(2)
 *          [3]    reserved    uint8
 *          [4..5] ch4         int16   ×1000
 *          [6..7] HP          uint16  裁判系统血量
 * @retval  0: 发送成功, 1: 发送失败
 */
uint8_t B2BSendKeysSwitch(void);

/**
 * @brief   发送机体姿态 + yaw 编码器帧
 * @note    帧格式（8 字节）：
 *          [0..1] roll_d × 100      int16  (0.01°)
 *          [2..3] pitch_d × 100     int16  (0.01°)
 *          [4..5] yaw_d × 100       int16  (0.01°)
 *          [6..7] yaw_enc_deg × 100 int16  (0.01°)
 * @param   roll_d       机体 roll  角 deg
 * @param   pitch_d      机体 pitch 角 deg
 * @param   yaw_d        机体 yaw   角 deg
 * @param   yaw_enc_deg  yaw 电机编码器角 deg
 * @retval  0: 发送成功, 1: 发送失败
 */
uint8_t B2BSendBodyState(float roll_d, float pitch_d, float yaw_d, float yaw_enc_deg);

/* ---- 上板→下板 接收 ---- */

/**
 * @brief   CAN 接收处理入口（收上板→下板帧）
 * @note    由 HAL_FDCAN_RxFifoCallback 调用
 * @param   can_id  CAN 帧 ID
 * @param   data    CAN 帧数据（8 字节）
 * @retval  1: 已处理, 0: 非本模块帧
 */
uint8_t B2BCanRxHandler(uint16_t can_id, uint8_t* data);

#endif /* _BOARD2BOARD_H_ */

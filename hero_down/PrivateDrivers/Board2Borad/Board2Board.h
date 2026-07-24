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
#define B2B_CAN                hfdcan3  /**< 双板通信用 CAN 总线（CAN3 独立通道，不抢电机帧优先级） */

/* ── 下板→上板 ───────────────────────────── */
#define B2B_DOWN_BODY_STATE    0x220U  /**< 机体姿态                   500Hz(2ms) */
#define B2B_DOWN_GIMBAL_INPUT  0x221U  /**< 云台目标(yaw_cmd+pitch_cmd) 100Hz(10ms) */
#define B2B_DOWN_KEYS_SWITCH   0x222U  /**< 键位 + 开关 + HP            100Hz(10ms,同RC) */
#define B2B_DOWN_STIR          0x223U  /**< 拨盘力矩+速度               500Hz(2ms) */
#define B2B_DOWN_SHOOT_STATE   0x224U  /**< 射击标志+弹速               100Hz(10ms) */

/* ── 上板→下板 ───────────────────────────── */
#define B2B_UP_GIMBAL_POSE     0x228U  /**< 云台姿态（高频）                 */
#define B2B_UP_GIMBAL_TARGET   0x229U  /**< 云台目标（低频）                 */
#define B2B_UP_FRIC_RPM_A      0x22AU  /**< 摩擦轮转速 0..3                  */
#define B2B_UP_FRIC_RPM_B      0x22BU  /**< 摩擦轮转速 4..5                  */
#define B2B_UP_GIMBAL_VELOCITY 0x22CU  /**< 云台 yaw/pitch 角速度（float）    */

/* ============================= API =================================== */

void B2BInit(void);

/* ---- 下板→上板 发送 ---- */

/**
 * @brief   发送云台控制输入帧（100Hz）
 * @note    yaw/pitch 最终目标角均使用 float，避免 0.01° 量化误差进入积分环。
 *          帧格式（8 字节）：
 *          [0..3] yaw_cmd         float  yaw 目标角 (deg)
 *          [4..7] pitch_cmd       float  pitch 目标角 (deg)
 * @param   yaw_cmd_d    yaw 目标角度 (deg)
 * @param   pitch_cmd_d  pitch 目标角度 (deg)
 * @retval  0: 发送成功, 1: 发送失败
 */
uint8_t B2BSendGimbalInput(float yaw_cmd_d, float pitch_cmd_d);

/**
 * @brief   发送射击状态帧（100Hz）
 * @note    [0] shoot_flag, [1..4] bullet_speed(float m/s), [5..7] reserved。
 */
uint8_t B2BSendShootState(void);

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
 * @brief   发送机体姿态帧
 * @note    帧格式（8 字节）：
 *          [0..1] roll_d  × 100   int16  (0.01°)
 *          [2..3] pitch_d × 100   int16  (0.01°)
 *          [4..5] yaw_d   × 100   int16  (0.01°)
 *          [6..7] 保留置零
 * @param   roll_d    机体 roll  角 deg
 * @param   pitch_d   机体 pitch 角 deg
 * @param   yaw_d     机体 yaw   角 deg
 * @retval  0: 发送成功, 1: 发送失败
 */
uint8_t B2BSendBodyState(float roll_d, float pitch_d, float yaw_d);

/**
 * @brief   发送拨盘数据帧（500Hz）
 * @note    帧格式（8 字节）：
 *          [0..1] stir_toq  int16  拨盘力矩 ×100 (Nm)
 *          [2..3] stir_vel  int16  拨盘速度 ×100 (rad/s)
 *          [4..7] 保留置零
 * @retval  0: 发送成功, 1: 发送失败
 */
uint8_t B2BSendStir(void);

/* ---- 上板→下板 接收 ---- */

/**
 * @brief   CAN 接收处理入口（收上板→下板帧）
 * @note    由 HAL_FDCAN_RxFifoCallback 调用
 * @param   can_id  CAN 帧 ID
 * @param   data    CAN 帧数据（8 字节）
 * @retval  1: 已处理, 0: 非本模块帧
 */
uint8_t B2BCanRxHandler(uint16_t can_id, uint8_t* data);

/**
 * @brief   B2B 云台姿态超时tick（由 GimbalPoseUpdate 每周期调用）
 * @note    递减心跳计数器，归零时自动清零 gimbal_yaw_rx_valid 触发安全回退
 */
extern volatile uint32_t can3_rx_isr_cnt;  /* CAN3 ISR触发计数，丢帧时看是否停 */
extern volatile uint16_t can3_last_rx_id;  /* CAN3最后收到的帧ID */
void B2B_PoseAliveTick(void);

#endif /* _BOARD2BOARD_H_ */

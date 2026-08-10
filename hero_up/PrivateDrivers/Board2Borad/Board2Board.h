/**
 * @file    Board2Board.h
 * @brief   双板 CAN 通信驱动（上板端）
 * @note    CAN: hfdcan1（双板直连），ID 段 0x220-0x22F。
 *          CAN 帧解析后直接写入 485 原有全局变量，消费端零改动。
 */

#ifndef _BOARD2BOARD_H_
#define _BOARD2BOARD_H_

#include "fdcan.h"
#include <stdint.h>

/* ===================================================================
 * CAN ID 分配 — B2B 专用段 0x220-0x22F (hfdcan3)
 *
 * 不与任何总线现有 ID 冲突，详见 hero_down/Board2Board.h
 * =================================================================== */
#define B2B_CAN hfdcan3 /**< 双板通信用 CAN 总线（CAN3 独立通道，不抢电机帧优先级） */

/* ── 下板→上板（收） ─────────────────────── */
#define B2B_DOWN_BODY_STATE 0x220U   /**< 机体姿态                          */
#define B2B_DOWN_GIMBAL_INPUT 0x221U /**< 云台目标(yaw_cmd+pitch_cmd) 100Hz */
#define B2B_DOWN_KEYS_SWITCH 0x222U
#define B2B_DOWN_STIR 0x223U        /**< 拨盘力矩+速度               500Hz */
#define B2B_DOWN_SHOOT_STATE 0x224U /**< 射击标志+弹速               100Hz */

/* ── 上板→下板（发） ─────────────────────── */
#define B2B_UP_GIMBAL_POSE 0x228U     /**< yaw/pitch姿态      500Hz(2ms)         */
#define B2B_UP_GIMBAL_TARGET 0x229U   /**< 候选yaw/pitch目标  100Hz(10ms)        */
#define B2B_UP_FRIC_RPM_A 0x22AU      /**< 摩擦轮 0..3        （暂不发送）        */
#define B2B_UP_FRIC_RPM_B 0x22BU      /**< 摩擦轮 4..5        （暂不发送）        */
#define B2B_UP_GIMBAL_VELOCITY 0x22CU /**< 云台角速度         500Hz(2ms)          */

/* ============================= API =================================== */

uint8_t B2BCanRxHandler(uint16_t can_id, uint8_t *data);

/* ---- B2B 下行（下板→上板）心跳监测 ---- */
extern volatile uint8_t g_b2b_down_valid; /* 0=下板B2B丢失，已进保护 */
void B2B_DownAliveCheck(void);            /* ControlTask每周期调用，递减计数器 */

/* ---- 上板→下板 发送 ---- */

uint8_t B2BSendGimbalPose(float yaw_d, float pitch_d, float yaw_dps, float pitch_dps);
uint8_t B2BSendGimbalTarget(float target_yaw, float target_pitch);
uint8_t B2BSendFricRPM(const int16_t rpm[6]);

#endif /* _BOARD2BOARD_H_ */

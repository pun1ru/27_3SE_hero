#ifndef DECISION_AO_H_
#define DECISION_AO_H_

#include "qpc_init.h"

/*--- 信号 ---*/
enum DecisionSig {
    /* 广播信号 (< MAX_PUB_SIG) */
    SWITCH_PC_SIG     = Q_USER_SIG,
    SWITCH_REMOTE_SIG,
    SWITCH_SNIPER_SIG,
    KEY_X_SIG,
    KEY_Q_SIG,
    KEY_F_SIG,
    KEY_G_SIG,
    KEY_C_SIG,
    KEY_V_SIG,
    KEY_B_SIG,
    KEY_E_SIG,
    KEY_Z_SIG,
    CH4_POS_SIG,
    CH4_NEG_SIG,
    L1_UP_SIG,
    L1_DOWN_SIG,
    MOUSE_LEFT_SIG,
    RC_LOST_SIG,
    MAX_PUB_SIG,

    /* 直发信号 (>= MAX_PUB_SIG) */
    TIMEOUT_SIG,
    AIM_DONE_SIG,

    MAX_SIG
};

/*--- 模式枚举 ---*/
enum CtrlTerminal { CTRL_STOP = 0, CTRL_REMOTE, CTRL_PC };
enum ChassisMode  { CHASSIS_FOLLOW = 0, CHASSIS_FOLLOW_BACK, CHASSIS_REVOLVE, CHASSIS_SEPARATE };

/*--- DecisionAO ---*/
typedef struct {
    QActive super;

    /* 外部只读 */
    uint8_t ctrl_terminal;
    uint8_t chassis_mode;
    uint8_t sniper;
    uint8_t fric_mode;
    uint8_t stir_mode;
    uint8_t joint_mode;
    uint8_t stand_mode;
    uint8_t mouse_fix;
    uint8_t cam_target;
    uint8_t world_enable;
    uint8_t stable;

    /* 内部 */
    uint8_t rc_lost_flag;

} DecisionAO;

extern DecisionAO  DecisionAO_inst;
extern QActive    * const AO_DecisionAO;
extern const DecisionAO * const _decisionAO;

/* 构造函数 */
void DecisionAO_ctor(void);

/* 状态函数 */
QState DecisionAO_initial      (DecisionAO * const me, void const * const par);
QState DecisionAO_Protected    (DecisionAO * const me, QEvt   const * const e);
QState DecisionAO_PcMode       (DecisionAO * const me, QEvt   const * const e);
QState DecisionAO_PcNormal     (DecisionAO * const me, QEvt   const * const e);
QState DecisionAO_PcSniper     (DecisionAO * const me, QEvt   const * const e);
QState DecisionAO_RemoteMode   (DecisionAO * const me, QEvt   const * const e);
QState DecisionAO_RemoteNormal (DecisionAO * const me, QEvt   const * const e);
QState DecisionAO_RemoteSniper (DecisionAO * const me, QEvt   const * const e);

#endif

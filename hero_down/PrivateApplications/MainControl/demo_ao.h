/**
 * @file    demo_ao.h
 * @brief   教学用：最简 QP 活动对象（Active Object）
 *
 * 这个 AO 模拟一个电灯开关，只有两个状态：OFF 和 ON
 * 你按一下 TOGGLE 事件 → OFF 变 ON，再按 → ON 变 OFF
 *
 * 看完这个示例你就理解了 QP 的四个核心元素：
 *   1. 信号（Signal）  — 就是事件的类型，比如 TOGGLE_SIG
 *   2. 事件（QEvt）    — 信号 + 参数，比如 "TOGGLE事件 + 按键编号=1"
 *   3. 状态（State）    — 状态机的节点，比如 OFF / ON
 *   4. 迁移（Transition）— 状态之间的箭头，比如 OFF → (收到TOGGLE) → ON
 */

#ifndef DEMO_AO_H_
#define DEMO_AO_H_

#include "qpc_init.h"   /* QP 框架入口 */

/*===========================================================================
 * 第1步：定义信号（Signal）
 *
 * 信号就是事件的 ID。QP 框架有自己的内部信号，
 * 你的应用信号必须从 Q_USER_SIG 开始编号，避免冲突。
 *
 * MAX_PUB_SIG 是 QP 发布-订阅系统的分界线：
 *   - 信号 < MAX_PUB_SIG    → 可以广播（publish），多个 AO 都能收到
 *   - 信号 >= MAX_PUB_SIG   → 只能直发（post）给一个 AO
 *
 * MAX_SIG 是整个枚举的结尾标志，框架用它来检查信号是否合法。
 *===========================================================================*/
enum DemoSig
{
    /*--- 可广播的信号（< MAX_PUB_SIG）---*/
    TOGGLE_SIG = Q_USER_SIG,  /* 总从 Q_USER_SIG 开始！ */
    TURN_ON_SIG,
    TURN_OFF_SIG,

    MAX_PUB_SIG,  /* ← 广播信号上限，必须在这 */

    /*--- 只能直发的信号（>= MAX_PUB_SIG）---*/
    TIMEOUT_SIG,  /* 定时器超时信号 */

    MAX_SIG       /* ← 所有信号上限，必须在这 */
};

/*===========================================================================
 * 第2步：定义 AO 类（就是你的活动对象）
 *
 * QActive  是 QP 提供的基类。你继承它，加上你自己的成员变量。
 * QTimeEvt 是 QP 的定时器事件，你可以在 AO 里放任意多个。
 *
 * 类比 C++：class DemoAO : public QActive { ... };
 * 这里用 C 的组合方式：QActive 作为第一个成员 = 继承
 *===========================================================================*/
typedef struct
{
    QActive super;            /* ★ QP 基类，必须放第一个！ */

    /* 你的私有成员 */
    uint8_t is_on;            /* 当前灯是否亮着 */
    uint8_t toggle_count;     /* 切换次数计数器 */

    QTimeEvt timeEvt;         /* QP 定时器（超时自动发 TIMEOUT_SIG） */

} DemoAO;

/* 全局 AO 实例 */
extern DemoAO demoAO;

/* AO 只读指针（其他模块通过它读取 AO 状态，不能写） */
extern const DemoAO * const _demoAO;

/*===========================================================================
 * 第3步：声明函数
 *
 * 每个 AO 至少需要：
 *   - 构造函数（ctor）   — 初始化 AO 和它的状态机
 *   - 初始状态函数       — 状态机的起点
 *   - 各状态的处理函数    — 每个状态一个函数
 *===========================================================================*/

/* 构造函数 */
void DemoAO_ctor(void);

/* 状态机状态函数（QP 会在合适的时机调用它们） */
QState DemoAO_initial(DemoAO * const me, void const * const par);
QState DemoAO_off(DemoAO * const me, QEvt const * const e);
QState DemoAO_on(DemoAO * const me, QEvt const * const e);

#endif /* DEMO_AO_H_ */

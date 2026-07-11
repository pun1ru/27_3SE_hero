/**
 * @file    demo_ao.c
 * @brief   教学用：最简 QP 状态机实现
 *
 * ┌────────── 状态图 ──────────┐
 * │                            │
 * │   ┌──────────┐  TOGGLE    ┌──────────┐
 * │   │   OFF    │ ────────→  │    ON    │
 * │   │ (初始状态) │ ←────────  │          │
 * │   └──────────┘  TOGGLE    └───┬──────┘
 * │                     或 TIMEOUT│
 * │                               │
 * │         每个箭头 = 一次"状态迁移"
 * └────────────────────────────┘
 *
 * QP 状态函数的签名规则（必须遵守！）：
 *   QState XxxStateName(YourAO * const me, QEvt const * const e)
 *                              ↑                  ↑
 *                          你的AO类型          收到的事件（可能为NULL）
 *
 * QP 状态函数返回值（二选一）：
 *   Q_HANDLED()  → "我处理了这个事件，但不切换状态"
 *   Q_TRAN(目标)  → "切换到目标状态"（比如 Q_TRAN(DemoAO_on)）
 *
 * ═══════════════ 这下面就是 QP 状态机的全部奥秘 ═══════════════
 */

#include "demo_ao.h"

/* 全局实例定义（头文件中 extern 的那个） */
DemoAO demoAO;
const DemoAO * const _demoAO = &demoAO;

/*===========================================================================
 * 构造函数 — 在 AO 启动前调用，做初始化
 *
 * 这里做的事情：
 *   1. QActive_ctor() → 告诉 QP 这个 AO 的初始状态是哪个
 *   2. QTimeEvt_ctorX() → 给 AO 装一个定时器，超时自动发事件给自己
 *===========================================================================*/
void DemoAO_ctor(void)
{
    DemoAO * const me = &demoAO;

    /* ★ 第一句必须写：注册初始状态 */
    QActive_ctor(&me->super, Q_STATE_CAST(&DemoAO_initial));

    /* ★ 注册定时器：超时发 TIMEOUT_SIG 到自己邮箱 */
    QTimeEvt_ctorX(&me->timeEvt,          /* 定时器变量            */
                   &me->super,            /* 所有者 = 这个 AO      */
                   TIMEOUT_SIG,           /* 超时发什么信号         */
                   0U);                   /* tickRate = 0（默认速率）*/

    /* 私有成员初始化 */
    me->is_on = 0U;
    me->toggle_count = 0U;
}

/*===========================================================================
 * 初始状态 — 状态机的起点，AO 启动时执行一次
 *
 * 注意：par 是启动参数（QACTIVE_START 的最后一个参数）
 *       这里不常用，但要知道它存在
 *===========================================================================*/
QState DemoAO_initial(DemoAO * const me, void const * const par)
{
    (void)par;   /* 不用参数，消灭编译警告 */

    /* 可以在这里做"开机动作"，比如蜂鸣器叫一声 */

    /* ★ 初始迁移：跳转到 OFF 状态 */
    return Q_TRAN(DemoAO_off);
}

/*===========================================================================
 * OFF 状态 — 灯灭
 *
 * 这个函数会被反复调用：每次有事件发给这个 AO，QP 就调它
 *===========================================================================*/
QState DemoAO_off(DemoAO * const me, QEvt const * const e)
{
    /* QP 的状态函数分两层：
     *   外层 switch (e->sig)   → 处理这个状态关心的信号
     *   内层 QState 返回值      → 告诉 QP 状态变没变
     */

    switch (e->sig)
    {
    /*--- 进入状态时（entry action）---*/
    case Q_ENTRY_SIG:
    {
        /* Q_ENTRY_SIG 是 QP 内部信号：每次进入这个状态时触发一次 */
        me->is_on = 0U;
        /* 实际项目里可以这里关灯，比如 HAL_GPIO_WritePin(LED, RESET); */
        return Q_HANDLED();  /* 处理完了，但不切换状态 */
    }

    /*--- 收到 TOGGLE 事件 → 切换到 ON ---*/
    case TOGGLE_SIG:
    {
        /* ★ 核心语法：Q_TRAN(目标状态函数) = 切换状态 */
        return Q_TRAN(DemoAO_on);
    }

    /*--- 收到 TURN_ON_SIG → 切换到 ON ---*/
    case TURN_ON_SIG:
    {
        return Q_TRAN(DemoAO_on);
    }

    /*--- 退出状态时（exit action）---*/
    case Q_EXIT_SIG:
    {
        /* Q_EXIT_SIG 在离开这个状态时触发一次 */
        /* 可以在这里做清理工作 */
        return Q_HANDLED();
    }

    default:
    {
        /* ★ 不认识的信号 → 交给 QP 基类处理（大部分情况就是这个）*/
        return Q_SUPER(&QHsm_top);
    }
    }
}

/*===========================================================================
 * ON 状态 — 灯亮
 *
 * 和 OFF 的区别：收到 TOGGLE 后回到 OFF
 *===========================================================================*/
QState DemoAO_on(DemoAO * const me, QEvt const * const e)
{
    switch (e->sig)
    {
    /*--- 进入 ON 状态 ---*/
    case Q_ENTRY_SIG:
    {
        me->is_on = 1U;
        ++me->toggle_count;  /* 每进一次 ON，计数器加1 */
        /* 实际项目里可以这里开灯 */

        /* ★ 设置定时器：3秒后自动发 TIMEOUT_SIG 给自己 */
        QTimeEvt_armX(&me->timeEvt,
                      (QTimeEvtCtr)(3U * 1000U),  /* 3秒 = 3000 tick（假设1ms/tick）*/
                      (QTimeEvtCtr)(3U * 1000U)); /* 周期 = 单次触发             */

        return Q_HANDLED();
    }

    /*--- 收到 TOGGLE → 回 OFF ---*/
    case TOGGLE_SIG:
    {
        return Q_TRAN(DemoAO_off);
    }

    /*--- 收到 TURN_OFF_SIG → 回 OFF ---*/
    case TURN_OFF_SIG:
    {
        return Q_TRAN(DemoAO_off);
    }

    /*--- 定时器超时 → 自动回 OFF ---*/
    case TIMEOUT_SIG:
    {
        /* ★ 收到定时器超时信号！熄灯 */
        return Q_TRAN(DemoAO_off);
    }

    /*--- 退出 ON 状态 ---*/
    case Q_EXIT_SIG:
    {
        /* 离开 ON 时取消定时器（避免在 OFF 状态下还收到超时） */
        QTimeEvt_disarm(&me->timeEvt);
        return Q_HANDLED();
    }

    default:
    {
        return Q_SUPER(&QHsm_top);
    }
    }
}

/*  ╔══════════════════════════════════════════════════════╗
 *  ║  恭喜！你看完了人生第一个 QP 状态机                    ║
 *  ║                                                      ║
 *  ║  总结三个最重要的宏：                                   ║
 *  ║    Q_TRAN(目标)  = 切换状态                             ║
 *  ║    Q_HANDLED()  = 处理完毕，不换状态                    ║
 *  ║    Q_SUPER(基类) = 交给父状态处理（不认识的事件）         ║
 *  ║                                                      ║
 *  ║  三个内置信号：                                         ║
 *  ║    Q_ENTRY_SIG  = 进入状态时触发（setup）               ║
 *  ║    Q_EXIT_SIG   = 离开状态时触发（cleanup）             ║
 *  ║    Q_INIT_SIG   = 初始迁移时触发                        ║
 *  ╚══════════════════════════════════════════════════════╝
 */

/*------------------------ 文件结束 -------------------------*/

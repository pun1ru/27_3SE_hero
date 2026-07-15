/**
 * @file    qpc_init.c
 * @brief   QP/C 胶水层 — QP 初始化 + 框架回调
 *
 * ┌────────── QP 启动流程 ──────────┐
 * │                                 │
 * │  QpInit()                       │
 * │    ├─ QF_init()        框架自检  │
 * │    ├─ QF_poolInit()    事件池    │
 * │    ├─ QActive_psInit() 订阅系统   │
 * │    ├─ DemoAO_ctor()    构造AO   │
 * │    └─ QACTIVE_START()  启动AO   │
 * │         │                       │
 * │         └─ 内部 xTaskCreate →   │
 * │            DemoAO 开始跑！      │
 * └─────────────────────────────────┘
 *
 * 对比 serialleg 的 Bsp_Start()：QpInit() 就是它的等价物。
 * 区别：serialleg 最后调 QF_run() 启动调度器，而本工程的
 * FreeRTOS 调度器已经在 main.c 中启动了，所以这里不需要 QF_run()。
 */

#include "qpc_init.h"
#include "decision_ao.h"     /* QM 生成的决策 AO */

/*===========================================================================
 * QpInit() — QP 框架启动（从 InitTask 末尾调用）
 *
 * 这个函数要做的事跟 serialleg 的 Bsp_Start() 一模一样，
 * 只是改了个名字（更清楚地表达"QP 初始化"）。
 *
 * 调用时机：InitTask 最后，所有硬件和普通 FreeRTOS 任务都创建完了之后。
 *===========================================================================*/
void QpInit(void)
{
    /* 严格对齐 serialleg Bsp_Start 顺序，一步不差 */

    /* 1. 事件池 */
    static QF_MPOOL_EL(QEvt) lPoolSto[100];
    QF_poolInit(lPoolSto, sizeof(lPoolSto), sizeof(lPoolSto[0]));

    /* 2. 发布-订阅 */
    static QSubscrList subscrSto[MAX_PUB_SIG];
    QActive_psInit(subscrSto, Q_DIM(subscrSto));

    /* 3. 构造 AO（serialleg: Class1_ctor） */
    DecisionAO_ctor();

    /* 4. 启动 AO（serialleg: pri=5, 我们: pri=5 同 serialleg） */
    static QEvt const *demoQueue[32] __attribute__((aligned(8))) = {0};
    static StackType_t demoStack[512] __attribute__((aligned(8)));

    QActive_setAttr(AO_DecisionAO, TASK_NAME_ATTR, "DecisionAO");
    QACTIVE_START(AO_DecisionAO,
                  5U,
                  demoQueue,
                  Q_DIM(demoQueue),
                  demoStack,
                  sizeof(demoStack),
                  (void *)0);

}

/*===========================================================================
 * 下面三个是 QP 框架的标准回调函数
 *
 * 当前工程中 FreeRTOS 调度器由 main.c 的 osKernelStart() 启动，
 * 不走 QF_run() 路径，所以 QF_onStartup / QF_onCleanup 不会被调用。
 * 但 QP 要求必须实现它们（否则链接报错），所以保留空实现。
 *===========================================================================*/

void QF_onStartup(void)
{
    /*
     * 如果将来改用 QF_run() 替代 vTaskStartScheduler()，
     * 把 InitTask 里的硬件初始化和 xTaskCreate 搬到这里。
     */
}

void QF_onCleanup(void)
{
    /* 正常比赛不会走到这 */
}

void Q_onError(char const *module, int_t id)
{
    /*
     * QP 内部断言失败时走到这里。
     * 可以先不管具体处理，死循环方便调试器断点定位。
     */
    for (;;)
    {
        /* TODO: 加 LED 闪烁或者安全关机 */
    }
}

/*------------------------ 文件结束 -------------------------*/

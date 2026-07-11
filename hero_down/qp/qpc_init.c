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
#include "demo_ao.h"     /* Demo 学习用 AO，后面换成你真正的决策 AO */

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
    /*--- 第1步：初始化事件池（QP 的动态内存） ---*/
    /*
     * 每个 QP AO 之间发消息用的是"事件"，事件有固定大小。
     * 事件池 = 预分配一坨内存块，需要发消息时从池子里拿一块，
     * 用完还回去。这样 rtos 运行时不 malloc，不会有碎片。
     *
     * lPoolSto[100] = 池子里最多100个事件同时在途
     * sizeof(lPoolSto[0]) = 每个存储单元的大小（QF_MPOOL_EL 已对齐过）
     */
    static QF_MPOOL_EL(QEvt) lPoolSto[100];
    QF_poolInit(lPoolSto, sizeof(lPoolSto), sizeof(lPoolSto[0]));

    /*--- 第2步：初始化发布-订阅系统 ---*/
    /*
     * 发布-订阅 = QP 的广播机制
     * 某个 AO 发广播 → 所有"订阅了该信号"的 AO 都能收到
     * 比如"遥控器丢信号了" → 云台AO和底盘AO都收到 → 各自做安全处理
     *
     * MAX_PUB_SIG = 可广播的信号上限（在 demo_ao.h 中定义）
     */
    static QSubscrList subscrSto[MAX_PUB_SIG];
    QActive_psInit(subscrSto, Q_DIM(subscrSto));

    /*--- 框架自检 ---*/
    QF_init();

    /*--- 第3步：构造 AO ---*/
    /*
     * 构造函数做两件事：
     *   1. QActive_ctor() → 告诉 QP "这个 AO 的初始状态函数是谁"
     *   2. QTimeEvt_ctorX() → 给 AO 装定时器
     */
    DemoAO_ctor();

    /*--- 第4步：启动 AO ---*/
    /*
     * QACTIVE_START() 内部做：
     *   1. xQueueCreateStatic() → 创建 FreeRTOS 消息队列（AO 的邮箱）
     *   2. xTaskCreateStatic()  → 创建 FreeRTOS 任务（AO 的执行线程）
     *
     * 参数说明：
     *   5U               → QP 优先级（数字越大优先级越高）
     *   demoQueue[32]    → 事件队列存储（最多积压32个事件）
     *   demoStack[512]   → FreeRTOS 任务栈
     *   NULL             → 启动参数（不需要）
     */
    static QEvt const *demoQueue[32];
    static StackType_t demoStack[512];

    QActive_setAttr(&demoAO.super, TASK_NAME_ATTR, "DemoAO");
    QACTIVE_START(&demoAO.super,
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

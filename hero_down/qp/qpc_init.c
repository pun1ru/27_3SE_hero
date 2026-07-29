#include "qpc_init.h"
#include "decision_ao.h"

#define DECISION_AO_PRIORITY 5U
#define DECISION_AO_QUEUE_LENGTH 32U
#define DECISION_AO_STACK_DEPTH 512U
#define QP_EVENT_POOL_LENGTH 100U

/**
 * @brief   初始化 QP/C 并启动下板决策主动对象
 * @param   void
 * @retval  void
 * @note    FreeRTOS 调度器已经运行，因此本函数不调用 QF_run()。
 */
void QpInit(void)
{
    static QF_MPOOL_EL(QEvt) event_pool[QP_EVENT_POOL_LENGTH];
    static QSubscrList subscriber_list[MAX_PUB_SIG];
    static QEvt const* decision_queue[DECISION_AO_QUEUE_LENGTH]
        __attribute__((aligned(8)));
    static StackType_t decision_stack[DECISION_AO_STACK_DEPTH]
        __attribute__((aligned(8)));

    QF_init();
    QF_poolInit(event_pool, sizeof(event_pool), sizeof(event_pool[0]));
    QActive_psInit(subscriber_list, Q_DIM(subscriber_list));

    DecisionAO_ctor();
    QActive_setAttr(AO_DecisionAO, TASK_NAME_ATTR, "DecisionAO");
    QACTIVE_START(AO_DecisionAO,
                  DECISION_AO_PRIORITY,
                  decision_queue,
                  Q_DIM(decision_queue),
                  decision_stack,
                  sizeof(decision_stack),
                  NULL);
}

void QF_onStartup(void)
{
}

void QF_onCleanup(void)
{
}

void Q_onError(char const* module, int_t id)
{
    (void)module;
    (void)id;

    taskDISABLE_INTERRUPTS();
    for(;;)
    {
    }
}

/**
 * @file task_decision.c
 * @brief DecisionTask — 输入决策任务，周期 10ms
 * @note  参照 hero_down 三段式架构，从 robot_control_task.c 拆分
 */

#include "task_decision.h"
#include "general_task_include.h"
#include "gimbalControl.h"
#include "shootControl.h"

static void DecisionInit(void);

void DecisionTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._decision_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._decision_task = &this_tick_count;

    DecisionInit();
    BulletKF_Init();

    current_tick_count = last_tick_count = xTaskGetTickCount();

    while (1)
    {
        /* 输入决策 */
        GimbalInputUpdate();
        ShootInputUpdate();

        /* 任务周期监控 */
        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;

        vTaskDelayUntil(&current_tick_count, DECISION_TASK_PERIOD_SET);
    }
}

static void DecisionInit(void)
{
    SmoothFilterInitialize(&MouseFilterX, 0.7);
    SmoothFilterInitialize(&MouseFilterY, 0.7);
}

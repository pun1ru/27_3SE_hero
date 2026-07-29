#include "task_decision.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "general_config_label.h"
#include "chassisControl.h"
#include "gimbalControl.h"
#include "jointControl.h"
#include "shoot_speed_best_contrl.h"
#include "stirControl.h"
#include "task_monitor.h"

/*============================================================================
 * DecisionTask — 输入决策任务
 * 周期: DECISION_TASK_PERIOD_SET (10ms)
 * 从遥控器/PC解析目标值，写入各控制模块的 InputTarget
 *============================================================================*/

/* 文件级辅助（仅 DecisionTask 内部使用） */
static void DecisionInit(void);

void DecisionTask(void* argument)
{
    TickType_t last_tick_count;

    (void)argument;

    DecisionInit();
    BulletKF_Init();

    last_tick_count = xTaskGetTickCount();

    while (1)
    {
        /* 输入决策 — 各函数实现已搬迁至 MainControl 模块文件 */
        ChassisInputUpdate();       /* chassisControl.c */
        JointInputUpdate();         /* jointControl.c */
        GimbalInputUpdate();        /* gimbalControl.c */
        ShootInputUpdate();         /* stirControl.c */

        /* 任务周期监控 */
        TickType_t current_tick_count = xTaskGetTickCount();
        TaskMonitorRecord(TASK_MONITOR_DECISION,
                          current_tick_count - last_tick_count);
        last_tick_count = current_tick_count;

        vTaskDelayUntil(&current_tick_count, DECISION_TASK_PERIOD_SET);
    }
}

/*============================================================================
 * 决策任务初始化
 *============================================================================*/
static void DecisionInit(void)
{
    GimbalDecisionInitialize();
    ChassisDecisionInitialize();
}

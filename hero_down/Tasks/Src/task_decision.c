#include "task_decision.h"
#include "general_task_include.h"

/*============================================================================
 * DecisionTask — 输入决策任务
 * 周期: DECISION_TASK_PERIOD_SET (10ms)
 * 从遥控器/PC解析目标值，写入各控制模块的 InputTarget
 *============================================================================*/

/* 文件级辅助（仅 DecisionTask 内部使用） */
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
        /* 输入决策 — 各函数实现已搬迁至 MainControl 模块文件 */
        ChassisInputUpdate();       /* chassisControl.c */
        JointInputUpdate();         /* jointControl.c */
        GimbalInputUpdate();        /* gimbalControl.c */
        ShootInputUpdate();         /* stirControl.c */

        /* 任务周期监控 */
        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;

        vTaskDelayUntil(&current_tick_count, DECISION_TASK_PERIOD_SET);
    }
}

/*============================================================================
 * 决策任务初始化
 *============================================================================*/
static void DecisionInit(void)
{
    extern SmoothFilter MouseFilterX;
    extern SmoothFilter MouseFilterY;
    SmoothFilterInitialize(&MouseFilterX, 0.7);
    SmoothFilterInitialize(&MouseFilterY, 0.7);

    PIDInitialize(&chassisControl.ChassisFollowControl.follow_speed_need_pid, -0.02, 0, 0.008, 0, 3);
    PIDInitialize(&chassisControl.ChassisFollowControl.follow_speed_need_pid, -0.02, 0, 0.008, 0, 3);
    PIDInitialize(&chassisControl.GimbalCoordinateInput.speed_x_compensate_pid, 0.1, 0.1, 0, 1, 3);
    PIDInitialize(&chassisControl.GimbalCoordinateInput.speed_y_compensate_pid, 0.1, 0.1, 0, 1, 3);
}

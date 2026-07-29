#include "task_estimate.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "chassisControl.h"
#include "gimbalControl.h"
#include "initial_task.h"
#include "jointControl.h"
#include "stirControl.h"
#include "task_monitor.h"

/*============================================================================
 * EstimateTask — 观测估计任务
 * 通知链: IMUTask ──Notify──▶ EstimateTask ──Notify──▶ ControlTask
 *============================================================================*/

void EstimateTask(void* argument)
{
    TickType_t last_tick_count;

    (void)argument;

    /* 清空初始化阶段可能残留的通知 */
    ulTaskNotifyTake(pdTRUE, 0);

    last_tick_count = xTaskGetTickCount();

    while (1)
    {
        /* 阻塞等待 IMUTask 通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 控制链监控：记录 Estimate 开始执行时刻 */
        TaskMonitorMarkEstimateEntry(DWT->CYCCNT);

        /* 观测更新 — 各函数实现已搬迁至 MainControl 模块文件 */
        ChassisEstimateUpdate();      /* chassisControl.c */
        JointEstimateUpdate();        /* jointControl.c */
        GimbalEstimateUpdate();       /* gimbalControl.c */
        ShootEstimateUpdate();        /* stirControl.c */

        /* 控制链监控：先完成 Estimate 打点，再唤醒 Control */
        TaskMonitorMarkEstimateExit(DWT->CYCCNT);

        /* 通知 ControlTask */
        if (controlTaskHandle != NULL)
        {
            xTaskNotifyGive(controlTaskHandle);
        }

        /* 任务周期监控 */
        {
            TickType_t current_tick_count = xTaskGetTickCount();
            TaskMonitorRecord(TASK_MONITOR_ESTIMATE,
                              current_tick_count - last_tick_count);
            last_tick_count = current_tick_count;
        }
    }
}

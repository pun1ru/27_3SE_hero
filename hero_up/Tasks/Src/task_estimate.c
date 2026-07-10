/**
 * @file task_estimate.c
 * @brief EstimateTask — 观测估计任务，通知驱动
 * @note  通知链: IMUTask ──Notify──▶ EstimateTask ──Notify──▶ ControlTask
 *         参照 hero_down 三段式架构
 */

#include "task_estimate.h"
#include "general_task_include.h"
#include "worldGimbal.h"
#include "gimbalControl.h"
#include "shootControl.h"

void EstimateTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count;
    static uint16_t task_counter;

    _taskMonitor->TaskFrameCounterPtr._estimate_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._estimate_task = &this_tick_count;

    /* 清空初始化阶段可能残留的通知 */
    ulTaskNotifyTake(pdTRUE, 0);

    current_tick_count = last_tick_count = xTaskGetTickCount();

    while (1)
    {
        /* 阻塞等待 IMUTask 通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 控制链监控：记录 Estimate 开始执行时刻 */
        g_chain_timer.cyc_est_entry = DWT->CYCCNT;

        /* 观测更新 */
        WorldGimbalEstimateUpdate(&worldGimbal);
        GimbalEstimateUpdate();
        ShootEstimateUpdate();

        /* 通知 ControlTask */
        extern TaskHandle_t controlTaskHandle;
        if (controlTaskHandle != NULL)
        {
            xTaskNotifyGive(controlTaskHandle);
        }

        /* 控制链监控：记录 Estimate 执行完毕时刻 */
        g_chain_timer.cyc_est_exit = DWT->CYCCNT;

        /* 任务周期监控 */
        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
    }
}

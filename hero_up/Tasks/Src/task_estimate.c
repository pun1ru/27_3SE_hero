#include "task_estimate.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "gimbalControl.h"
#include "initial_task.h"
#include "shootControl.h"
#include "task_monitor.h"
#include "worldGimbal.h"

void EstimateTask(void *argument){
    TickType_t last_tick_count;

    (void)argument;

    ulTaskNotifyTake(pdTRUE, 0U);
    last_tick_count = xTaskGetTickCount();

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        TaskMonitorMarkEstimateEntry(DWT->CYCCNT);

        WorldGimbalEstimateUpdate();
        GimbalEstimateUpdate();
        ShootEstimateUpdate();

        TaskMonitorMarkEstimateExit(DWT->CYCCNT);

        if(controlTaskHandle != NULL){
            xTaskNotifyGive(controlTaskHandle);
        }

        {
            TickType_t current_tick_count = xTaskGetTickCount();
            TaskMonitorRecord(TASK_MONITOR_ESTIMATE,
                              current_tick_count - last_tick_count);
            last_tick_count = current_tick_count;
        }
    }
}

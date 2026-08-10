#include "task_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "gimbalControl.h"
#include "shootControl.h"
#include "task_monitor.h"
#include "task_transmit.h"
#include "worldGimbal.h"

static void control_initialize(void);

void ControlTask(void *argument){
    TickType_t last_tick_count;

    (void)argument;

    control_initialize();
    last_tick_count = xTaskGetTickCount();
    vTaskDelay(400U);

    ulTaskNotifyTake(pdTRUE, 0U);
    last_tick_count = xTaskGetTickCount();

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        TaskMonitorMarkControlEntry(DWT->CYCCNT);

        WorldGimbalIKSolve();
        WorldGimbalApplyToTargets();
        GimbalControlUpdate();
        ShootControlUpdate();
        MotorControlCANSend();

        TaskMonitorMarkControlExit(DWT->CYCCNT);

        {
            TickType_t current_tick_count = xTaskGetTickCount();
            TaskMonitorRecord(TASK_MONITOR_CONTROL,
                              current_tick_count - last_tick_count);
            last_tick_count = current_tick_count;
        }
    }
}

static void control_initialize(void){
    GimbalControlInitialize();
    ShootControlInitialize();
}

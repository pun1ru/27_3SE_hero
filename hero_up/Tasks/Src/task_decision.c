#include "task_decision.h"

#include "FreeRTOS.h"
#include "task.h"

#include "general_config_label.h"
#include "gimbalControl.h"
#include "shootControl.h"
#include "shoot_speed_best_contrl.h"
#include "task_monitor.h"

void DecisionTask(void *argument){
    TickType_t last_tick_count;

    (void)argument;

    BulletKF_Init();
    last_tick_count = xTaskGetTickCount();

    while(1){
        TickType_t current_tick_count;

        GimbalInputUpdate();
        ShootInputUpdate();

        current_tick_count = xTaskGetTickCount();
        TaskMonitorRecord(TASK_MONITOR_DECISION,
                          current_tick_count - last_tick_count);
        last_tick_count = current_tick_count;

        vTaskDelayUntil(&current_tick_count, DECISION_TASK_PERIOD_SET);
    }
}

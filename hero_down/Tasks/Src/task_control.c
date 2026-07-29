#include "task_control.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "chassisControl.h"
#include "DMJ4310.h"
#include "gimbalControl.h"
#include "jointControl.h"
#include "peripheral_transmit_task.h"
#include "stirControl.h"
#include "task_monitor.h"

/*============================================================================
 * ControlTask — 闭环控制+输出任务
 * 通知链: EstimateTask ──Notify──▶ ControlTask
 *============================================================================*/

static void ControlInit(void)
{
    ChassisControlInitialize();

    GimbalControlInitialize();

    ShootControlInitialize();

    /* 关节力控 */
    JointForceControlInit(30, 0.003);
    JointForceControlTuningParamInit();
}

void ControlTask(void* argument)
{
    TickType_t last_tick_count;

    (void)argument;

    ControlInit();
    last_tick_count = xTaskGetTickCount();
    vTaskDelay(400);

    /* 启动电机 */
    start_motor(&hfdcan1, GMJ4310MOTOR_ID);
    vTaskDelay(200);

    /* 清空残留通知 */
    ulTaskNotifyTake(pdTRUE, 0);
    last_tick_count = xTaskGetTickCount();

    while (1)
    {
        /* 阻塞等待 EstimateTask 通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 控制链监控：记录 Control 开始执行时刻 */
        TaskMonitorMarkControlEntry(DWT->CYCCNT);

        /* 闭环控制 — 各函数实现已搬迁至 MainControl 模块文件 */
        ChassisControlUpdate();      /* chassisControl.c */
        GimbalControlUpdate();       /* gimbalControl.c */
        ShootControlUpdate();        /* stirControl.c */
        JointControlUpdate();        /* jointControl.c */

        /* 统一 CAN 发送 */
        MotorControlCANSend();       /* peripheral_transmit_task.c */

        /* 控制链监控：记录 Control 执行完毕 + 刷新各段微秒延迟 */
        TaskMonitorMarkControlExit(DWT->CYCCNT);

        /* 任务周期监控 */
        {
            TickType_t current_tick_count = xTaskGetTickCount();
            TaskMonitorRecord(TASK_MONITOR_CONTROL,
                              current_tick_count - last_tick_count);
            last_tick_count = current_tick_count;
        }
    }
}

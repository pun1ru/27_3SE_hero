#include "task_control.h"
#include "general_task_include.h"
#include "LK_driver.h"
#include "DMJ4310.h"
#include "CAN_driver.h"

/*============================================================================
 * ControlTask — 闭环控制+输出任务
 * 通知链: EstimateTask ──Notify──▶ ControlTask
 *============================================================================*/

extern float kpfric;
extern float kdfric;

static void ControlInit(void)
{
    ChassisControlInitialize();

    /* 云台 LADRC 初始化 */
    GimbalInit();

    /* 拨盘参数 */
    shootControl.ShootMotorControl.stir_preset_angle = STIR_PRESET_ANGLE;
    shootControl.ShootTargetInput.stir_target_vol = STIR_MAX_SPEED;
    shootControl.ShootEstimate.stir_enableflag_desire = DISABLE;

    /* 关节力控 */
    JointForceControlInit(30, 0.003);
    JointForceControlTuningParamInit();
}

void ControlTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._control_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._control_task = &this_tick_count;

    ControlInit();
    current_tick_count = last_tick_count = xTaskGetTickCount();
    vTaskDelay(400);

    /* 启动电机 */
    start_motor(&hfdcan1, GMJ4310MOTOR_ID);
    vTaskDelay(200);

    /* 清空残留通知 */
    ulTaskNotifyTake(pdTRUE, 0);

    while (1)
    {
        /* 阻塞等待 EstimateTask 通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 控制链监控：记录 Control 开始执行时刻 */
        g_chain_timer.cyc_ctrl_entry = DWT->CYCCNT;

        /* 闭环控制 — 各函数实现已搬迁至 MainControl 模块文件 */
        ChassisControlUpdate();      /* chassisControl.c */
        GimbalControlUpdate();       /* gimbalControl.c */
        ShootControlUpdate();        /* stirControl.c */
        JointControlUpdate();        /* jointControl.c */

        /* 统一 CAN 发送 */
        MotorControlCANSend();       /* peripheral_transmit_task.c */

        /* 控制链监控：记录 Control 执行完毕 + 刷新各段微秒延迟 */
        g_chain_timer.cyc_ctrl_exit = DWT->CYCCNT;
        g_chain_timer.imu_to_est_us  = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_est_entry  - g_chain_timer.cyc_imu_notify);
        g_chain_timer.est_exec_us    = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_est_exit   - g_chain_timer.cyc_est_entry);
        g_chain_timer.est_to_ctrl_us = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_ctrl_entry - g_chain_timer.cyc_est_exit);
        g_chain_timer.ctrl_exec_us   = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_ctrl_exit  - g_chain_timer.cyc_ctrl_entry);
        g_chain_timer.chain_total_us = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_ctrl_exit  - g_chain_timer.cyc_imu_notify);
        if (g_chain_timer.chain_total_us > g_chain_timer.chain_max_us)
            g_chain_timer.chain_max_us = g_chain_timer.chain_total_us;

        /* 任务周期监控 */
        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
    }
}

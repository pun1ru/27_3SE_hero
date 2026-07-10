/**
 * @file task_control.c
 * @brief ControlTask — 闭环控制+CAN发送任务，通知驱动
 * @note  通知链: EstimateTask ──Notify──▶ ControlTask
 *         参照 hero_down 三段式架构
 */

#include "task_control.h"
#include "general_task_include.h"
#include "worldGimbal.h"
#include "gimbalControl.h"
#include "shootControl.h"
#include "peripheral_transmit_task.h"
#include "LK_driver.h"
#include "DMJ4310.h"
#include "CAN_driver.h"

extern float kpfric;
extern float kdfric;

static void ControlInit(void);

void ControlTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._control_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._control_task = &this_tick_count;

    ControlInit();
    current_tick_count = last_tick_count = xTaskGetTickCount();
    vTaskDelay(400);

    /* 清空残留通知 */
    ulTaskNotifyTake(pdTRUE, 0);

    while (1)
    {
        /* 阻塞等待 EstimateTask 通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 控制链监控：记录 Control 开始执行时刻 */
        g_chain_timer.cyc_ctrl_entry = DWT->CYCCNT;

        /* 世界系 IK 反解 + 应用到目标（需要最新的估计结果） */
        WorldGimbalIKSolve(&worldGimbal);
        WorldGimbalApplyToTargets(&worldGimbal);

        /* 闭环控制 */
        GimbalControlUpdate();
        ShootControlUpdate();

        /* 统一 CAN 发送 */
        MotorControlCANSend();

        /* 控制链监控：记录 Control 完毕 + 刷新各段微秒延迟 */
        g_chain_timer.cyc_ctrl_exit = DWT->CYCCNT;
        g_chain_timer.imu_to_est_us  = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_est_entry  - g_chain_timer.cyc_imu_notify);
        g_chain_timer.est_exec_us    = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_est_exit   - g_chain_timer.cyc_est_entry);
        g_chain_timer.est_to_ctrl_us = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_ctrl_entry - g_chain_timer.cyc_est_exit);
        g_chain_timer.ctrl_exec_us   = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_ctrl_exit  - g_chain_timer.cyc_ctrl_entry);
        g_chain_timer.chain_total_us = CTRL_CHAIN_CYC_TO_US(g_chain_timer.cyc_ctrl_exit  - g_chain_timer.cyc_imu_notify);
        if (g_chain_timer.chain_total_us > g_chain_timer.chain_max_us)
            g_chain_timer.chain_max_us = g_chain_timer.chain_total_us;

        /* 控制周期：距上次 Control 退出的时间 */
        if (g_chain_timer.cyc_last_ctrl_exit != 0)
        {
            g_chain_timer.ctrl_period_us = CTRL_CHAIN_CYC_TO_US(
                g_chain_timer.cyc_ctrl_exit - g_chain_timer.cyc_last_ctrl_exit);
            if (g_chain_timer.ctrl_period_us > g_chain_timer.ctrl_period_max_us)
                g_chain_timer.ctrl_period_max_us = g_chain_timer.ctrl_period_us;
        }
        g_chain_timer.cyc_last_ctrl_exit = g_chain_timer.cyc_ctrl_exit;

        /* 任务周期监控 */
        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
    }
}

static void ControlInit(void)
{
    /* 云台 ADRC/LTD 初始化 */
    GimbalInit();

    /* 摩擦轮 PID 初始化 */
    PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[LEFT]),  kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
    PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[RIGHT]), kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
    PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[UP]),    kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
    PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[LEFT1]),  kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
    PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[RIGHT1]), kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
    PIDInitialize(&(shootControl.ShootMotorControl.fric_speed_pid[UP1]),    kpfric, 0, kdfric, 0, TEMP_SHOOT_3508_CURRENT_MAX);
}

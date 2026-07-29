#include "task_monitor.h"
#include "event_groups.h"
#include "iwdg.h"
#include "peripheral_receive_task.h"
#include "general_config_label.h"

extern EventGroupHandle_t remoteRecEventGroup;

#define TASK_MONITOR_PERIOD_TOLERANCE_TICKS 2U
#define REMOTE_WARNING_LIMIT_CYCLES 3U
#define REMOTE_BLOCK_LIMIT_CYCLES 10U
#define CTRL_CHAIN_CYCLES_PER_US 480U

static volatile TaskMonitor task_monitor;
const volatile TaskMonitor* const _taskMonitor = &task_monitor;

static const TickType_t expected_period_ticks[TASK_MONITOR_COUNT] =
{
    [TASK_MONITOR_REMOTE_RECEIVE] = 0U,
    [TASK_MONITOR_STATE] = STATE_TASK_PERIOD_SET,
    [TASK_MONITOR_DECISION] = DECISION_TASK_PERIOD_SET,
    [TASK_MONITOR_CONTROL] = CONTROL_TASK_PERIOD_SET,
    [TASK_MONITOR_IMU] = IMU_TASK_PERIOD_SET,
    [TASK_MONITOR_DEBUG] = DEBUG_TASK_PERIOD_SET,
    [TASK_MONITOR_UPPER_COMM] = UPPER_COMM_TASK_PERIOD_SET,
    [TASK_MONITOR_UI_OPERATION] = UI_OPERATION_TASK_PERIOD_SET,
    [TASK_MONITOR_MUSIC] = MUSIC_TASK_PERIOD_SET,
    [TASK_MONITOR_ESTIMATE] = ESTIMATE_TASK_PERIOD_SET,
};

static uint32_t task_monitor_cycles_to_us(uint32_t cycle_count)
{
    return cycle_count / CTRL_CHAIN_CYCLES_PER_US;
}

static void task_monitor_set_fault(TaskMonitorId task_id, uint8_t has_fault)
{
    uint16_t mask = TASK_MONITOR_MASK(task_id);

    if(has_fault != 0U)
    {
        task_monitor.fault_mask |= mask;
    }
    else
    {
        task_monitor.fault_mask &= (uint16_t)(~mask);
    }
}

static void task_monitor_check_remote(uint32_t last_frame_count)
{
    static uint8_t no_receive_count;
    static uint8_t blocked_count;
    uint32_t current_frame_count = task_monitor.task[TASK_MONITOR_REMOTE_RECEIVE].frame_count;

    if(current_frame_count == last_frame_count)
    {
        if(no_receive_count < UINT8_MAX)
        {
            no_receive_count++;
        }
        if(blocked_count < UINT8_MAX)
        {
            blocked_count++;
        }
    }
    else
    {
        no_receive_count = 0U;
        blocked_count = 0U;
    }

    if(no_receive_count > REMOTE_WARNING_LIMIT_CYCLES)
    {
        no_receive_count = 0U;
        if(remoteRecEventGroup != NULL)
        {
            xEventGroupSetBits(remoteRecEventGroup, EVENT_GROUP_BIT_ERROR);
        }
    }

    task_monitor_set_fault(TASK_MONITOR_REMOTE_RECEIVE,
                           blocked_count > REMOTE_BLOCK_LIMIT_CYCLES);
}

static void task_monitor_check_periodic_task(TaskMonitorId task_id,
                                             uint32_t last_frame_count)
{
    const volatile TaskRuntimeSample* sample = &task_monitor.task[task_id];
    uint8_t is_blocked;
    uint8_t period_disturbed;

    if(sample->frame_count == 0U)
    {
        task_monitor_set_fault(task_id, 0U);
        return;
    }

    is_blocked = (sample->frame_count == last_frame_count);
    period_disturbed = sample->period_ticks
                       > (expected_period_ticks[task_id]
                          + TASK_MONITOR_PERIOD_TOLERANCE_TICKS);
    task_monitor_set_fault(task_id, is_blocked || period_disturbed);
}

static void task_monitor_check_tasks(uint32_t last_frame_count[TASK_MONITOR_COUNT])
{
    TaskMonitorId task_id;

    task_monitor_check_remote(last_frame_count[TASK_MONITOR_REMOTE_RECEIVE]);

    for(task_id = TASK_MONITOR_STATE; task_id < TASK_MONITOR_COUNT; task_id++)
    {
        task_monitor_check_periodic_task(task_id, last_frame_count[task_id]);
    }

    for(task_id = TASK_MONITOR_REMOTE_RECEIVE;
        task_id < TASK_MONITOR_COUNT;
        task_id++)
    {
        last_frame_count[task_id] = task_monitor.task[task_id].frame_count;
    }
}

void TaskMonitorRecord(TaskMonitorId task_id, TickType_t period_ticks)
{
    if(task_id >= TASK_MONITOR_COUNT)
    {
        return;
    }

    task_monitor.task[task_id].period_ticks = period_ticks;
    task_monitor.task[task_id].frame_count++;
}

void TaskMonitorMarkImuNotify(uint32_t cycle_count)
{
    task_monitor.control_chain.cyc_imu_notify = cycle_count;
}

void TaskMonitorMarkEstimateEntry(uint32_t cycle_count)
{
    task_monitor.control_chain.cyc_est_entry = cycle_count;
}

void TaskMonitorMarkEstimateExit(uint32_t cycle_count)
{
    task_monitor.control_chain.cyc_est_exit = cycle_count;
}

void TaskMonitorMarkControlEntry(uint32_t cycle_count)
{
    task_monitor.control_chain.cyc_ctrl_entry = cycle_count;
}

void TaskMonitorMarkControlExit(uint32_t cycle_count)
{
    volatile CtrlChainTimer* timer = &task_monitor.control_chain;

    timer->cyc_ctrl_exit = cycle_count;
    timer->imu_to_est_us = task_monitor_cycles_to_us(timer->cyc_est_entry
                                                      - timer->cyc_imu_notify);
    timer->est_exec_us = task_monitor_cycles_to_us(timer->cyc_est_exit
                                                    - timer->cyc_est_entry);
    timer->est_to_ctrl_us = task_monitor_cycles_to_us(timer->cyc_ctrl_entry
                                                       - timer->cyc_est_exit);
    timer->ctrl_exec_us = task_monitor_cycles_to_us(timer->cyc_ctrl_exit
                                                     - timer->cyc_ctrl_entry);
    timer->chain_total_us = task_monitor_cycles_to_us(timer->cyc_ctrl_exit
                                                       - timer->cyc_imu_notify);
    if(timer->chain_total_us > timer->chain_max_us)
    {
        timer->chain_max_us = timer->chain_total_us;
    }
}

void MonitorTask(void* argument)
{
    uint32_t last_frame_count[TASK_MONITOR_COUNT] = {0U};
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)argument;

    while(1)
    {
        HAL_IWDG_Refresh(&hiwdg1);
        task_monitor_check_tasks(last_frame_count);
        vTaskDelayUntil(&last_wake_tick, MONITOR_TASK_PERIOD_SET);
    }
}

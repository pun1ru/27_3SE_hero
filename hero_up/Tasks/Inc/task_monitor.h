#ifndef TASK_MONITOR_H_
#define TASK_MONITOR_H_

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

typedef enum {
    TASK_MONITOR_REMOTE_RECEIVE = 0,
    TASK_MONITOR_STATE,
    TASK_MONITOR_DECISION,
    TASK_MONITOR_CONTROL,
    TASK_MONITOR_IMU,
    TASK_MONITOR_DEBUG,
    TASK_MONITOR_UPPER_COMM,
    TASK_MONITOR_MUSIC,
    TASK_MONITOR_ESTIMATE,
    TASK_MONITOR_COUNT
} TaskMonitorId;

#define TASK_MONITOR_MASK(task_id) ((uint16_t)(1U << (task_id)))

typedef struct {
    uint32_t frame_count;
    TickType_t period_ticks;
} TaskRuntimeSample;

typedef struct {
    uint32_t cyc_imu_notify;
    uint32_t cyc_est_entry;
    uint32_t cyc_est_exit;
    uint32_t cyc_ctrl_entry;
    uint32_t cyc_ctrl_exit;
    uint32_t imu_to_est_us;
    uint32_t est_exec_us;
    uint32_t est_to_ctrl_us;
    uint32_t ctrl_exec_us;
    uint32_t chain_total_us;
    uint32_t chain_max_us;
} CtrlChainTimer;

typedef struct {
    TaskRuntimeSample task[TASK_MONITOR_COUNT];
    CtrlChainTimer control_chain;
    uint16_t fault_mask;
} TaskMonitor;

extern const volatile TaskMonitor *const _taskMonitor;
extern QueueHandle_t g_musicQueue;

/**
 * @brief   Record one task iteration and its measured period
 * @param   task_id Task monitor identifier
 * @param   period_ticks Measured period in RTOS ticks
 * @retval  void
 */
void TaskMonitorRecord(TaskMonitorId task_id, TickType_t period_ticks);

/**
 * @brief   Record the cycle count when IMU notifies EstimateTask
 * @param   cycle_count DWT cycle count
 * @retval  void
 */
void TaskMonitorMarkImuNotify(uint32_t cycle_count);

/**
 * @brief   Record the cycle count when EstimateTask starts
 * @param   cycle_count DWT cycle count
 * @retval  void
 */
void TaskMonitorMarkEstimateEntry(uint32_t cycle_count);

/**
 * @brief   Record the cycle count when EstimateTask finishes
 * @param   cycle_count DWT cycle count
 * @retval  void
 */
void TaskMonitorMarkEstimateExit(uint32_t cycle_count);

/**
 * @brief   Record the cycle count when ControlTask starts
 * @param   cycle_count DWT cycle count
 * @retval  void
 */
void TaskMonitorMarkControlEntry(uint32_t cycle_count);

/**
 * @brief   Record the cycle count when ControlTask finishes
 * @param   cycle_count DWT cycle count
 * @retval  void
 */
void TaskMonitorMarkControlExit(uint32_t cycle_count);

void MonitorTask(void *argument);
void MusicTask(void *argument);

#endif

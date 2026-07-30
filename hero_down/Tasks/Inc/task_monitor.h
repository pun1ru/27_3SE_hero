#ifndef TASK_MONITOR_H_
#define TASK_MONITOR_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdint.h>

typedef enum{
    TASK_MONITOR_REMOTE_RECEIVE = 0,
    TASK_MONITOR_STATE,
    TASK_MONITOR_DECISION,
    TASK_MONITOR_CONTROL,
    TASK_MONITOR_IMU,
    TASK_MONITOR_DEBUG,
    TASK_MONITOR_UPPER_COMM,
    TASK_MONITOR_UI_OPERATION,
    TASK_MONITOR_MUSIC,
    TASK_MONITOR_ESTIMATE,
    TASK_MONITOR_COUNT
} TaskMonitorId;

#define TASK_MONITOR_MASK(task_id) ((uint16_t)(1U << (task_id)))

typedef struct{
    uint32_t frame_count;
    TickType_t period_ticks;
} TaskRuntimeSample;

typedef struct{
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

typedef struct{
    TaskRuntimeSample task[TASK_MONITOR_COUNT];
    CtrlChainTimer control_chain;
    uint16_t fault_mask;
} TaskMonitor;

extern const volatile TaskMonitor* const _taskMonitor;
extern QueueHandle_t g_musicQueue;

/**
 * @brief   记录任务完成一次循环及其实际周期
 * @param   task_id 任务监控编号
 * @param   period_ticks 本次循环周期，单位为 RTOS tick
 * @retval  void
 */
void TaskMonitorRecord(TaskMonitorId task_id, TickType_t period_ticks);

/**
 * @brief   记录 IMU 发出 Estimate 通知的时刻
 * @param   cycle_count DWT 周期计数
 * @retval  void
 */
void TaskMonitorMarkImuNotify(uint32_t cycle_count);

/**
 * @brief   记录 Estimate 开始执行的时刻
 * @param   cycle_count DWT 周期计数
 * @retval  void
 */
void TaskMonitorMarkEstimateEntry(uint32_t cycle_count);

/**
 * @brief   记录 Estimate 完成执行的时刻
 * @param   cycle_count DWT 周期计数
 * @retval  void
 */
void TaskMonitorMarkEstimateExit(uint32_t cycle_count);

/**
 * @brief   记录 Control 开始执行的时刻
 * @param   cycle_count DWT 周期计数
 * @retval  void
 */
void TaskMonitorMarkControlEntry(uint32_t cycle_count);

/**
 * @brief   记录 Control 完成执行的时刻并更新控制链耗时
 * @param   cycle_count DWT 周期计数
 * @retval  void
 */
void TaskMonitorMarkControlExit(uint32_t cycle_count);

/**
 * @brief   任务和链路看门狗任务
 * @param   argument 未使用
 * @retval  void
 */
void MonitorTask(void* argument);

/**
 * @brief   Buzzer playback and circuit-error monitoring task
 * @param   argument Unused
 * @retval  void
 */
void MusicTask(void* argument);

#endif

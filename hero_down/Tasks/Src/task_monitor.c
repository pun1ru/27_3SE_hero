#include "task_monitor.h"
#include "task_monitor_internal.h"

#include <stdint.h>

#include "event_groups.h"
#include "main.h"
#include "iwdg.h"
#include "music_mardio.h"
#include "task_receive.h"
#include "tim.h"

#include "device_define.h"
#include "general_define.h"

extern EventGroupHandle_t remoteRecEventGroup;

/* ============================ 监控配置 ============================ */
#define TASK_MONITOR_PERIOD_TOLERANCE_TICKS 2U
#define REMOTE_WARNING_LIMIT_CYCLES 3U
#define REMOTE_BLOCK_LIMIT_CYCLES 10U
#define CTRL_CHAIN_CYCLES_PER_US 480U

/* ========================== 监控运行状态 ========================== */
static volatile TaskMonitor task_monitor;
const volatile TaskMonitor* const _taskMonitor = &task_monitor;

static const TickType_t expected_period_ticks[TASK_MONITOR_COUNT] = {
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

static CircuitMonitor circuit_monitor;

/* ======================== 监控内部辅助函数 ======================== */
static uint32_t task_monitor_cycles_to_us(uint32_t cycle_count){
    return cycle_count / CTRL_CHAIN_CYCLES_PER_US;
}

static void task_monitor_set_fault(TaskMonitorId task_id, uint8_t has_fault){
    uint16_t mask = TASK_MONITOR_MASK(task_id);

    if(has_fault != 0U){
        task_monitor.fault_mask |= mask;
    }
    else{
        task_monitor.fault_mask &= (uint16_t)(~mask);
    }
}

static void task_monitor_check_remote(uint32_t last_frame_count){
    static uint8_t no_receive_count;
    static uint8_t blocked_count;
    uint32_t current_frame_count = task_frame_count(TASK_MONITOR_REMOTE_RECEIVE);

    if(current_frame_count == last_frame_count){
        if(no_receive_count < UINT8_MAX){
            no_receive_count++;
        }
        if(blocked_count < UINT8_MAX){
            blocked_count++;
        }
    }
    else{
        no_receive_count = 0U;
        blocked_count = 0U;
    }

    if(no_receive_count > REMOTE_WARNING_LIMIT_CYCLES){
        no_receive_count = 0U;
        if(remoteRecEventGroup != NULL){
            xEventGroupSetBits(remoteRecEventGroup, EVENT_GROUP_BIT_ERROR);
        }
    }

    task_monitor_set_fault(TASK_MONITOR_REMOTE_RECEIVE,
                           blocked_count > REMOTE_BLOCK_LIMIT_CYCLES);
}

static void task_monitor_check_periodic_task(TaskMonitorId task_id,
                                             uint32_t last_frame_count){
    uint8_t is_blocked;
    uint8_t period_disturbed;

    if(task_frame_count(task_id) == 0U){
        task_monitor_set_fault(task_id, 0U);
        return;
    }

    is_blocked = (task_frame_count(task_id) == last_frame_count);
    period_disturbed = task_period_ticks(task_id)
                       > (expected_period_ticks[task_id]
                          + TASK_MONITOR_PERIOD_TOLERANCE_TICKS);
    task_monitor_set_fault(task_id, is_blocked || period_disturbed);
}

static void task_monitor_check_tasks(uint32_t last_frame_count[TASK_MONITOR_COUNT]){
    TaskMonitorId task_id;

    task_monitor_check_remote(last_frame_count[TASK_MONITOR_REMOTE_RECEIVE]);

    for(task_id = TASK_MONITOR_STATE; task_id < TASK_MONITOR_COUNT; task_id++){
        task_monitor_check_periodic_task(task_id, last_frame_count[task_id]);
    }

    for(task_id = TASK_MONITOR_REMOTE_RECEIVE;
        task_id < TASK_MONITOR_COUNT;
        task_id++){
        last_frame_count[task_id] = task_frame_count(task_id);
    }
}

/* ======================== 音乐播放辅助函数 ======================== */
static void music_startup_notice(const float* score, int note_count,
                                 int wait_time, float pitch_offset){
    int note_index;

    music_init(BUZZER_TIM, BUZZER_TIM_CHANNEL);
    for(note_index = 0; note_index < note_count; note_index++){
        play_music(from_notes_to_pr(score[note_index] + pitch_offset),
                   wait_time,
                   BUZZER_TIM);
    }
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0U);
}

static void music_check_circuit_error(void){
    int motor_index;

    for(motor_index = 0; motor_index < 4; motor_index++){
        chassis_error[motor_index] = chassis_frame_count[motor_index] == _chassisMotorRec[motor_index].frame_counter;
        joint_error[motor_index] = joint_frame_count[motor_index] == _jointMotorEec[motor_index].frame_counter;
    }

    yaw_error = (gimbal_yaw_rx_valid == 0U);

    for(motor_index = 0; motor_index < 2; motor_index++){
        caterpillar_error[motor_index] = caterpillar_frame_count[motor_index] == _caterpillarMotorRec[motor_index].frame_counter;
    }

    stir_error = (stir_frame_count == _stirMotorRec->frame_counter);
    super_capacity_error = (super_capacity_frame_count == _superCapacity->frame_counter);

    for(motor_index = 0; motor_index < 4; motor_index++){
        chassis_frame_count[motor_index] = _chassisMotorRec[motor_index].frame_counter;
        joint_frame_count[motor_index] = _jointMotorEec[motor_index].frame_counter;
    }
    for(motor_index = 0; motor_index < 2; motor_index++){
        caterpillar_frame_count[motor_index] = _caterpillarMotorRec[motor_index].frame_counter;
    }
    stir_frame_count = _stirMotorRec->frame_counter;
    super_capacity_frame_count = _superCapacity->frame_counter;
}

/* ========================= 监控公共接口 ========================= */
void TaskMonitorRecord(TaskMonitorId task_id, TickType_t period_ticks){
    if(task_id >= TASK_MONITOR_COUNT){
        return;
    }

    task_period_ticks(task_id) = period_ticks;
    task_frame_count(task_id)++;
}

void TaskMonitorMarkImuNotify(uint32_t cycle_count){
    task_monitor.control_chain.cyc_imu_notify = cycle_count;
}

void TaskMonitorMarkEstimateEntry(uint32_t cycle_count){
    task_monitor.control_chain.cyc_est_entry = cycle_count;
}

void TaskMonitorMarkEstimateExit(uint32_t cycle_count){
    task_monitor.control_chain.cyc_est_exit = cycle_count;
}

void TaskMonitorMarkControlEntry(uint32_t cycle_count){
    task_monitor.control_chain.cyc_ctrl_entry = cycle_count;
}

void TaskMonitorMarkControlExit(uint32_t cycle_count){
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
    if(timer->chain_total_us > timer->chain_max_us){
        timer->chain_max_us = timer->chain_total_us;
    }
}

/* ======================== 看门狗与任务健康监控 ======================== */
void MonitorTask(void* argument){
    uint32_t last_frame_count[TASK_MONITOR_COUNT] = {0U};
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)argument;

    while(1){
        HAL_IWDG_Refresh(&hiwdg1);
        task_monitor_check_tasks(last_frame_count);
        vTaskDelayUntil(&last_wake_tick, MONITOR_TASK_PERIOD_SET);
    }
}

/* ======================== 蜂鸣器播放与电路监测 ======================== */
void MusicTask(void* argument){
    TickType_t last_tick_count;
    TickType_t current_tick_count;
    int16_t music_receive;

    (void)argument;
    current_tick_count = xTaskGetTickCount();
    last_tick_count = current_tick_count;
    while(1){
        music_check_circuit_error();
        if(xQueueReceive(g_musicQueue, &music_receive, 0U) == pdPASS){
            switch(music_receive){
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    music_startup_notice(alone_earth,
                                         (int)(sizeof(alone_earth) / sizeof(alone_earth[0])),
                                         180,
                                         -8.0f);
                    break;
                default:
                    music_startup_notice(bleach,
                                         (int)(sizeof(bleach) / sizeof(bleach[0])),
                                         100,
                                         -8.0f);
                    break;
            }
        }

        current_tick_count = xTaskGetTickCount();
        TaskMonitorRecord(TASK_MONITOR_MUSIC,
                          current_tick_count - last_tick_count);
        last_tick_count = current_tick_count;
        vTaskDelayUntil(&current_tick_count, MUSIC_TASK_PERIOD_SET);
    }
}

/* ======================= FreeRTOS 异常回调钩子 ======================= */
void vApplicationStackOverflowHook(TaskHandle_t x_task, char* task_name){
    (void)x_task;
    (void)task_name;
}

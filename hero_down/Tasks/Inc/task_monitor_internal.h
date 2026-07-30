#ifndef TASK_MONITOR_INTERNAL_H_
#define TASK_MONITOR_INTERNAL_H_

#include <stdint.h>

typedef struct{
    struct{
        uint16_t chassis_motor_frame_counter[4];
        uint16_t yaw_motor_frame_counter;
        uint16_t stir_motor_frame_counter;
        uint16_t joint_motor_frame_counter[4];
        uint16_t caterpillar_motor_frame_counter[2];
        uint16_t super_capacity_frame_counter;
    } frame_counter;
    struct{
        uint8_t chassis_motor_error[4];
        uint8_t yaw_motor_error;
        uint8_t stir_motor_error;
        uint8_t joint_motor_error[4];
        uint8_t caterpillar_motor_error[2];
        uint8_t super_capacity_error;
    } error;
} CircuitMonitor;

#define chassis_frame_count circuit_monitor.frame_counter.chassis_motor_frame_counter
#define joint_frame_count circuit_monitor.frame_counter.joint_motor_frame_counter
#define caterpillar_frame_count circuit_monitor.frame_counter.caterpillar_motor_frame_counter
#define stir_frame_count circuit_monitor.frame_counter.stir_motor_frame_counter
#define super_capacity_frame_count circuit_monitor.frame_counter.super_capacity_frame_counter
#define chassis_error circuit_monitor.error.chassis_motor_error
#define yaw_error circuit_monitor.error.yaw_motor_error
#define joint_error circuit_monitor.error.joint_motor_error
#define caterpillar_error circuit_monitor.error.caterpillar_motor_error
#define stir_error circuit_monitor.error.stir_motor_error
#define super_capacity_error circuit_monitor.error.super_capacity_error
#define task_frame_count(task_id) task_monitor.task[(task_id)].frame_count
#define task_period_ticks(task_id) task_monitor.task[(task_id)].period_ticks

#endif

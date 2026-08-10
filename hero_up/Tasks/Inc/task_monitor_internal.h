#ifndef TASK_MONITOR_INTERNAL_H_
#define TASK_MONITOR_INTERNAL_H_

#include <stdint.h>

typedef struct {
    struct {
        uint16_t yaw_motor_frame_count;
        uint16_t fric_motor_frame_count[6];
        uint16_t pitch_motor_frame_count;
    } frame_count;
    struct {
        uint8_t yaw_motor_error;
        uint8_t fric_motor_error[6];
        uint8_t pitch_motor_error;
    } error;
} CircuitMonitor;

#define yaw_frame_count circuit_monitor.frame_count.yaw_motor_frame_count
#define fric_frame_count circuit_monitor.frame_count.fric_motor_frame_count
#define pitch_frame_count circuit_monitor.frame_count.pitch_motor_frame_count
#define yaw_error circuit_monitor.error.yaw_motor_error
#define fric_error circuit_monitor.error.fric_motor_error
#define pitch_error circuit_monitor.error.pitch_motor_error
#define task_frame_count(task_id) task_monitor.task[(task_id)].frame_count
#define task_period_ticks(task_id) task_monitor.task[(task_id)].period_ticks

#endif

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
    /* 底盘电机 PID */
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[LF]), 8, 0, 2.0, 0, M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[RF]), 8, 0, 2.0, 0, M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[RB]), 7, 0, 1.5, 0, M3508_MAX_OUTPUT_CURRENT);
    PIDInitialize(&(chassisControl.WheelMotorControl.speed_control_pid[LB]), 7, 0, 1.5, 0, M3508_MAX_OUTPUT_CURRENT);

    /* 云台 LADRC 初始化 */
    GimbalInit();

    /* 拨盘参数 */
    shootControl.ShootMotorControl.stir_preset_angle = STIR_PRESET_ANGLE;
    shootControl.ShootTargetInput.stir_target_vol = STIR_MAX_SPEED;
    shootControl.ShootEstimate.stir_enableflag_desire = DISABLE;

    /* 功率滤波器 */
    extern AverageFilter PowerFilter;
    AverageFilterInitialize(&PowerFilter);

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
    uint8_t adata[8];
    LK_Motor_run(adata);
    CANTransmit_U8(&hfdcan2, 0x141, adata);
    CANTransmit_U8(&hfdcan3, 0x142, adata);
    vTaskDelay(200);

    /* 清空残留通知 */
    ulTaskNotifyTake(pdTRUE, 0);

    while (1)
    {
        /* 阻塞等待 EstimateTask 通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 闭环控制 — 各函数实现已搬迁至 MainControl 模块文件 */
        ChassisControlUpdate();      /* chassisControl.c */
        GimbalControlUpdate();       /* gimbalControl.c */
        ShootControlUpdate();        /* stirControl.c */
        JointControlUpdate();        /* jointControl.c */

        /* 统一 CAN 发送 */
        MotorControlCANSend();       /* peripheral_transmit_task.c */

        /* 任务周期监控 */
        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
    }
}

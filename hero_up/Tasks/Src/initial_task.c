#include "initial_task.h"

#include <stdint.h>

#include "main.h"
#include "queue.h"

#include "bsp_dwt.h"
#include "general_define.h"
#include "qpc_init.h"
#include "task_control.h"
#include "task_decision.h"
#include "task_estimate.h"
#include "task_monitor.h"
#include "task_receive.h"
#include "task_state.h"
#include "task_transmit.h"
#include "tim.h"
#include "WS2812.h"

TaskHandle_t decisionTaskHandle;
TaskHandle_t stateMachineTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t estimateTaskHandle;
TaskHandle_t upperPCCommTaskHandle;
TaskHandle_t remoteRecTaskHandle;
TaskHandle_t monitorTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t debugTaskHandle;
TaskHandle_t musicTaskHandle;

QueueHandle_t g_musicQueue;

static void create_task(TaskFunction_t task_function,
                        const char *task_name,
                        configSTACK_DEPTH_TYPE stack_depth,
                        UBaseType_t priority,
                        TaskHandle_t *task_handle);

/**
 * @brief   Initialize board resources and create hero_up tasks
 * @param   argument Unused
 * @retval  void
 */
void InitTask(void const *argument){
    (void)argument;

    DWT_Init(480U);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    WS2812_PWM_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    WS2812_SPI_Ctrl(50U, 0U, 0U);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500U);

    taskENTER_CRITICAL();

    g_musicQueue = xQueueCreate(1U, sizeof(int16_t));
    configASSERT(g_musicQueue != NULL);

    WS2812_SPI_Ctrl(25U, 25U, 0U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    RemoteRecInitialize();
    QpInit();
    PeripheralRecEnable();

    create_task(MonitorTask, "MonitorTask", 128U, 7U, &monitorTaskHandle);
    create_task(RemoteRecTask, "RemoteRecTask", 512U, 7U,
                &remoteRecTaskHandle);
    create_task(StateMachineTask, "StateMachineTask", 2048U, 7U,
                &stateMachineTaskHandle);
    create_task(DecisionTask, "DecisionTask", 512U, 5U,
                &decisionTaskHandle);
    create_task(EstimateTask, "EstimateTask", 512U, 5U,
                &estimateTaskHandle);
    create_task(ControlTask, "ControlTask", 512U, 5U, &controlTaskHandle);
    create_task(IMUTask, "IMUTask", 512U, 5U, &imuTaskHandle);
    create_task(DebugTask, "DebugTask", 256U, 4U, &debugTaskHandle);
    create_task(UpperPCCommTask, "UpperPCCommTask", 256U, 5U,
                &upperPCCommTaskHandle);
    create_task(MusicTask, "MusicTask", 512U, 2U, &musicTaskHandle);

    WS2812_SPI_Ctrl(0U, 10U, 0U);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);

    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}

static void create_task(TaskFunction_t task_function,
                        const char *task_name,
                        configSTACK_DEPTH_TYPE stack_depth,
                        UBaseType_t priority,
                        TaskHandle_t *task_handle){
    BaseType_t result = xTaskCreate(task_function,
                                    task_name,
                                    stack_depth,
                                    NULL,
                                    priority,
                                    task_handle);

    configASSERT(result == pdPASS);
    if(result != pdPASS){
        Error_Handler();
    }
}

#include "initial_task.h"

#include "main.h"
#include "queue.h"

#include "device_define.h"
#include "bsp_dwt.h"
#include "music_mardio.h"
#include "task_receive.h"
#include "task_transmit.h"
#include "qpc_init.h"
#include "task_state.h"
#include "task_control.h"
#include "task_decision.h"
#include "task_estimate.h"
#include "task_monitor.h"
#include "tim.h"

TaskHandle_t decisionTaskHandle;
TaskHandle_t stateMachineTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t estimateTaskHandle;
TaskHandle_t upperPCCommTaskHandle;
TaskHandle_t remoteRecTaskHandle;
TaskHandle_t uiOperationTaskHandle;
TaskHandle_t monitorTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t debugTaskHandle;
TaskHandle_t musicTaskHandle;

QueueHandle_t g_musicQueue;

static void startup_notice(void);
static void create_task(TaskFunction_t task_function,
                        const char* task_name,
                        configSTACK_DEPTH_TYPE stack_depth,
                        UBaseType_t priority,
                        TaskHandle_t* task_handle);

/**
 * @brief   初始化板级运行资源并创建 hero_down 任务
 * @param   argument 未使用
 * @retval  void
 */
void InitTask(void const* argument)
{
    (void)argument;

    DWT_Init(480U);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    startup_notice();
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500U);

    taskENTER_CRITICAL();

    g_musicQueue = xQueueCreate(1U, sizeof(int16_t));
    configASSERT(g_musicQueue != NULL);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    RemoteRecInitialize();
    PeripheralRecEnable();
    QpInit();

    create_task(MonitorTask, "MonitorTask", 128U, 4U, &monitorTaskHandle);
    create_task(RemoteRecTask, "RemoteRecTask", 256U, 7U, &remoteRecTaskHandle);
    create_task(StateMachineTask, "StateMachineTask", 2048U, 6U,
                &stateMachineTaskHandle);
    create_task(DecisionTask, "DecisionTask", 512U, 5U, &decisionTaskHandle);
    create_task(EstimateTask, "EstimateTask", 512U, 5U, &estimateTaskHandle);
    create_task(ControlTask, "ControlTask", 512U, 5U, &controlTaskHandle);
    create_task(IMUTask, "IMUTask", 512U, 7U, &imuTaskHandle);
    create_task(DebugTask, "DebugTask", 256U, 4U, &debugTaskHandle);
    create_task(UIOperationTask, "UIOperationTask", 512U, 3U,
                &uiOperationTaskHandle);
    create_task(UpperPCCommTask, "UpperPCCommTask", 256U, 5U,
                &upperPCCommTaskHandle);
    create_task(MusicTask, "MusicTask", 512U, 2U, &musicTaskHandle);

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);

    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}

static void create_task(TaskFunction_t task_function,
                        const char* task_name,
                        configSTACK_DEPTH_TYPE stack_depth,
                        UBaseType_t priority,
                        TaskHandle_t* task_handle)
{
    BaseType_t result = xTaskCreate(task_function,
                                    task_name,
                                    stack_depth,
                                    NULL,
                                    priority,
                                    task_handle);

    configASSERT(result == pdPASS);
    if(result != pdPASS)
    {
        Error_Handler();
    }
}

static void startup_notice(void)
{
    int note_index;
    int note_count;

#define STARTUP_MELODY alone_earth

    music_init(BUZZER_TIM, BUZZER_TIM_CHANNEL);
    note_count = (int)(sizeof(STARTUP_MELODY) / sizeof(STARTUP_MELODY[0]));
    for(note_index = 0; note_index < note_count; note_index++)
    {
        play_music(from_notes_to_pr(STARTUP_MELODY[note_index] - 8.0f),
                   100,
                   BUZZER_TIM);
    }

#undef STARTUP_MELODY
}

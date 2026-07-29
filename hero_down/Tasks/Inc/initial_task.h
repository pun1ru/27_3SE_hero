#ifndef INITIAL_TASK_H_
#define INITIAL_TASK_H_

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t decisionTaskHandle;
extern TaskHandle_t stateMachineTaskHandle;
extern TaskHandle_t controlTaskHandle;
extern TaskHandle_t estimateTaskHandle;
extern TaskHandle_t upperPCCommTaskHandle;
extern TaskHandle_t remoteRecTaskHandle;
extern TaskHandle_t uiOperationTaskHandle;
extern TaskHandle_t monitorTaskHandle;
extern TaskHandle_t imuTaskHandle;
extern TaskHandle_t debugTaskHandle;
extern TaskHandle_t musicTaskHandle;

void InitTask(void const* argument);

#endif

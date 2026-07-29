#ifndef MUSIC_TASK_H_
#define MUSIC_TASK_H_

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t g_musicQueue;

void MusicTask(void* argument);

#endif

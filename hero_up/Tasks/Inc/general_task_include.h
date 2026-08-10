#ifndef GENERAL_TASK_INCLUDE_H_
#define GENERAL_TASK_INCLUDE_H_

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "freertos.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "general_config_label.h"
#include "general_define.h"

#include "initial_task.h"
#include "task_control.h"
#include "task_decision.h"
#include "task_estimate.h"
#include "task_monitor.h"
#include "task_receive.h"
#include "task_state.h"
#include "task_transmit.h"

#include "decision_ao.h"
#include "qpc_init.h"

#include "distance_check.h"
#include "judge_receive.h"

#include "adrc.h"
#include "algorism.h"
#include "arm_math.h"
#include "MadWick.h"
#include "pid.h"
#include "shoot_speed_best_contrl.h"

#include "CAN_driver.h"
#include "DMJ4310.h"
#include "gimbalControl.h"
#include "shootControl.h"
#include "worldGimbal.h"

#endif

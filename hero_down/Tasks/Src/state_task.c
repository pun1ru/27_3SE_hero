#include "state_task.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "decision_ao.h"
#include "general_config_label.h"
#include "judge_receive.h"
#include "peripheral_receive_task.h"
#include "stirControl.h"
#include "task_monitor.h"

static void state_update(const NormRemoteCmd* command,
                         NormRemoteCmd* last_command);
static void decision_input_post(const NormRemoteCmd* command,
                                const NormRemoteCmd* last_command);
static void decision_reset_outputs(DecisionAO* decision);
static uint8_t key_rising_edge(uint8_t key, uint8_t last_key);
static void post_decision_event(const QEvt* event);

static RobotState robot_state;
const RobotState* const _robotState = &robot_state;

int crawler_rotate_flag;

static NormRemoteCmd last_remote_command;

static const QEvt event_switch_protected = {.sig = SWITCH_FINAL_SIG};
static const QEvt event_switch_pc = {.sig = Switch_PC_SIG};
static const QEvt event_switch_remote = {.sig = Switch_RC_SIG};
static const QEvt event_switch_sniper = {.sig = SWITCH_SNIPER_SIG};
static const QEvt event_switch_revolve = {.sig = SWITCH_REVOLVE_SIG};
static const QEvt event_switch_back = {.sig = SWITCH_BACK_SIG};
static const QEvt event_switch_stair = {.sig = SWITCH_STAIR_SIG};
static const QEvt event_rc_lost = {.sig = RC_LOST_SIG};

/**
 * @brief   解析遥控输入并向决策主动对象投递状态事件
 * @param   argument 未使用
 * @retval  void
 */
void StateMachineTask(void* argument)
{
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t last_sample_tick = last_wake_tick;

    (void)argument;

    while(1)
    {
        TickType_t current_tick;

        state_update(_normRemoteCmd, &last_remote_command);

        current_tick = xTaskGetTickCount();
        TaskMonitorRecord(TASK_MONITOR_STATE,
                          current_tick - last_sample_tick);
        last_sample_tick = current_tick;
        vTaskDelayUntil(&last_wake_tick, STATE_TASK_PERIOD_SET);
    }
}

static uint8_t key_rising_edge(uint8_t key, uint8_t last_key)
{
    return (key != 0U) && (last_key == 0U);
}

static void post_decision_event(const QEvt* event)
{
    QACTIVE_POST(AO_DecisionAO, event, NULL);
}

static void decision_reset_outputs(DecisionAO* decision)
{
    decision->ctrl_terminal = CTRL_STOP;
    decision->chassis_mode = CHS_FOLLOW;
    decision->sniper = SNIPER_OFF;
    decision->fric_mode = FRIC_OFF;
    decision->stir_mode = STIR_LOCK;
    decision->joint_mode = JOINT_NORMAL;
    decision->stand_mode = STAND_NORMAL;
    decision->mouse_fix = MOUSE_FIX_OFF;
    decision->cam_target = CAM_TARGET_MID;
    decision->world_enable = WORLD_ENABLE_OFF;
    decision->can_enable = CAN_DISABLE;
}

static void decision_post_terminal_event(uint8_t switch_left,
                                         uint8_t switch_right,
                                         uint8_t last_switch_left,
                                         uint8_t last_switch_right)
{
    if((switch_right == last_switch_right)
       && (switch_left == last_switch_left))
    {
        return;
    }

    if((switch_left == NORM_RC_SW_MID)
       && (switch_right == NORM_RC_SW_UP))
    {
        post_decision_event(&event_switch_pc);
    }
    else
    {
        post_decision_event(&event_switch_remote);
    }
}

static uint8_t decision_post_keyboard_events(const NormRemoteCmd* command,
                                             const NormRemoteCmd* last_command)
{
    DecisionAO* decision = &DecisionAO_inst;

    if(key_rising_edge(command->PCKeyBoard.level_key_X,
                       last_command->PCKeyBoard.level_key_X))
    {
        post_decision_event(&event_switch_sniper);
    }
    if(key_rising_edge(command->PCKeyBoard.level_key_Q,
                       last_command->PCKeyBoard.level_key_Q))
    {
        post_decision_event(&event_switch_revolve);
    }
    if(key_rising_edge(command->PCKeyBoard.level_key_G,
                       last_command->PCKeyBoard.level_key_G))
    {
        post_decision_event(&event_switch_back);
    }
    if(key_rising_edge(command->PCKeyBoard.level_key_V,
                       last_command->PCKeyBoard.level_key_V))
    {
        post_decision_event(&event_switch_stair);
    }
    if(key_rising_edge(command->PCKeyBoard.level_key_F,
                       last_command->PCKeyBoard.level_key_F))
    {
        decision->fric_mode = (decision->fric_mode == FRIC_ON)
                              ? FRIC_OFF
                              : FRIC_ON;
    }
    if(key_rising_edge(command->PCKeyBoard.level_key_E,
                       last_command->PCKeyBoard.level_key_E))
    {
        decision->mouse_fix = (decision->mouse_fix == MOUSE_FIX_ON)
                              ? MOUSE_FIX_OFF
                              : MOUSE_FIX_ON;
    }
    if(key_rising_edge(command->PCKeyBoard.level_key_Z,
                       last_command->PCKeyBoard.level_key_Z))
    {
        if(decision->cam_target == CAM_TARGET_MID)
        {
            decision->cam_target = CAM_TARGET_UP;
        }
        else if(decision->cam_target == CAM_TARGET_UP)
        {
            decision->cam_target = CAM_TARGET_DOWN;
        }
        else
        {
            decision->cam_target = CAM_TARGET_MID;
        }
    }

    return decision->fric_mode;
}

static uint8_t decision_post_remote_fric_event(uint8_t switch_left,
                                               uint8_t switch_right,
                                               uint8_t last_switch_left,
                                               uint8_t fric_mode)
{
    DecisionAO* decision = &DecisionAO_inst;

    if((switch_left == NORM_RC_SW_UP)
       && (switch_left != last_switch_left)
       && !((switch_left == NORM_RC_SW_UP)
            && (switch_right == NORM_RC_SW_UP)))
    {
        fric_mode = (fric_mode == FRIC_ON) ? FRIC_OFF : FRIC_ON;
        decision->fric_mode = fric_mode;
    }

    return fric_mode;
}

static void decision_post_stir_event(const NormRemoteCmd* command,
                                     uint8_t fric_mode,
                                     uint8_t is_pc_mode)
{
    DecisionAO* decision = &DecisionAO_inst;

    if(is_pc_mode == 0U)
    {
        if((command->Switch.switch_L1 == NORM_RC_SW_DOWN)
           && (fric_mode != FRIC_OFF))
        {
            decision->stir_mode = STIR_ANGLE_CONTROL;
        }
        else if(_shootControl->ShootEstimate.stir_block_flag == 0U)
        {
            decision->stir_mode = STIR_LOCK;
        }
    }

    if(command->PCMouse.mouse_left
       && (fric_mode == FRIC_ON)
       && ((ext_game_robot_status.shooter_barrel_heat_limit
            - ext_power_heat_data.shooter_42mm_barrel_heat) >= 100))
    {
        decision->stir_mode = STIR_ANGLE_CONTROL;
    }

    if(_shootControl->ShootEstimate.stir_block_flag != 0U)
    {
        decision->stir_mode = STIR_REVERSE;
    }
}

static void decision_input_post(const NormRemoteCmd* command,
                                const NormRemoteCmd* last_command)
{
    uint8_t switch_left = command->Switch.switch_L1;
    uint8_t switch_right = command->Switch.switch_R1;
    uint8_t last_switch_left = last_command->Switch.switch_L1;
    uint8_t last_switch_right = last_command->Switch.switch_R1;
    uint8_t fric_mode;
    uint8_t is_pc_mode;

    if((switch_left == NORM_RC_SW_UP)
       && (switch_right == NORM_RC_SW_UP))
    {
        decision_reset_outputs(&DecisionAO_inst);
        post_decision_event(&event_switch_protected);
        return;
    }

    decision_post_terminal_event(switch_left,
                                 switch_right,
                                 last_switch_left,
                                 last_switch_right);

    fric_mode = decision_post_keyboard_events(command, last_command);
    fric_mode = decision_post_remote_fric_event(switch_left,
                                                 switch_right,
                                                 last_switch_left,
                                                 fric_mode);
    is_pc_mode = (switch_left == NORM_RC_SW_MID)
                 && (switch_right == NORM_RC_SW_UP);
    decision_post_stir_event(command, fric_mode, is_pc_mode);

    if((is_pc_mode == 0U)
       && (((switch_right == NORM_RC_SW_DOWN)
            && (last_switch_right != NORM_RC_SW_DOWN))
           || ((switch_right != NORM_RC_SW_DOWN)
               && (last_switch_right == NORM_RC_SW_DOWN))))
    {
        post_decision_event(&event_switch_sniper);
    }
}

static void state_update(const NormRemoteCmd* command,
                         NormRemoteCmd* last_command)
{
    switch(command->remote_source)
    {
        case DT7:
        {
            DecisionAO_inst.rc_lost_flag = RC_OK;
            decision_input_post(command, last_command);
            break;
        }

        case VT13:
        {
            break;
        }

        case ERROR_RECEIVE:
        default:
        {
            DecisionAO_inst.rc_lost_flag = RC_LOST;
            post_decision_event(&event_rc_lost);
            break;
        }
    }

    memcpy(last_command, command, sizeof(*last_command));
}

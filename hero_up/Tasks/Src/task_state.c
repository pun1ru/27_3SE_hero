#include "task_state.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "decision_ao.h"
#include "distance_check.h"
#include "general_config_label.h"
#include "gimbalControl.h"
#include "judge_receive.h"
#include "task_monitor.h"
#include "task_receive.h"
#include "task_transmit.h"
#include "worldGimbal.h"

#define KEY_Q PCKeyBoard.level_key_Q
#define KEY_E PCKeyBoard.level_key_E
#define KEY_R PCKeyBoard.level_key_R
#define KEY_F PCKeyBoard.level_key_F
#define KEY_Z PCKeyBoard.level_key_Z
#define KEY_X PCKeyBoard.level_key_X
#define KEY_B PCKeyBoard.level_key_B

#define KEY_UPTRIG(key) ((command->key != 0U) && (last_command->key == 0U))
#define VALUE_UPTRIG(value, last_value) ((value) && !(last_value))
#define VALUE_DOWNTRIG(value, last_value) (!(value) && (last_value))

#define STATE_TASK_FREQUENCY_HZ 100.0f
#define WORLD_GIMBAL_PRESET_PITCH_D 40.0f
#define CAMERA_LAYOUT_FIRST 0x01U
#define CAMERA_LAYOUT_LAST 0x04U
#define UPPER_PC_SHUTDOWN_REQUEST 0x01U

static void state_update(const NormRemoteCmd *command,
                         NormRemoteCmd *last_command);
static void decision_input_post(const NormRemoteCmd *command,
                                const NormRemoteCmd *last_command,
                                uint8_t force_terminal_event);
static void post_decision_event(const QEvt *event);
static void virtual_heat_update(void);

static NormRemoteCmd last_remote_command;

static const QEvt event_switch_protected = {.sig = Switch_protected_SIG};
static const QEvt event_switch_pc = {.sig = Switch_PC_SIG};
static const QEvt event_switch_remote = {.sig = Switch_RC_SIG};
static const QEvt event_switch_sniper = {.sig = SWITCH_SNIPER_SIG};
static const QEvt event_switch_world = {.sig = SWITCH_WORLD_SIG};
static const QEvt event_rc_lost = {.sig = RC_LOST_SIG};

uint16_t xvni;

extern UpperComputerComm upperComputerComm;

/**
 * @brief   Parse operator input and update the upper-board decision AO
 * @param   argument Unused
 * @retval  void
 */
void StateMachineTask(void *argument){
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t last_sample_tick = last_wake_tick;

    (void)argument;

    while(1){
        TickType_t current_tick;

        state_update(_normRemoteCmd, &last_remote_command);

        current_tick = xTaskGetTickCount();
        TaskMonitorRecord(TASK_MONITOR_STATE,
                          current_tick - last_sample_tick);
        last_sample_tick = current_tick;
        vTaskDelayUntil(&last_wake_tick, STATE_TASK_PERIOD_SET);
    }
}

static void post_decision_event(const QEvt *event){
    QACTIVE_POST(AO_DecisionAO, event, NULL);
}

static void decision_post_terminal_event(uint8_t is_pc_mode,
                                         uint8_t switch_left,
                                         uint8_t switch_right,
                                         uint8_t last_switch_left,
                                         uint8_t last_switch_right,
                                         uint8_t force_terminal_event){
    if((force_terminal_event == 0U) &&
       (switch_right == last_switch_right) &&
       (switch_left == last_switch_left)){
        return;
    }

    if(is_pc_mode != 0U){
        post_decision_event(&event_switch_pc);
    } else {
        post_decision_event(&event_switch_remote);
    }
}

static void decision_handle_pc_input(const NormRemoteCmd *command,
                                     const NormRemoteCmd *last_command){
    DecisionAO *const decision = &DecisionAO_inst;

    if(KEY_UPTRIG(KEY_X)){
        uint8_t enter_sniper = (pDecisionAO->sniper == SNIPER_OFF);

        post_decision_event(&event_switch_sniper);
        if(enter_sniper != 0U){
            post_decision_event(&event_switch_world);
        }
    }
    if(KEY_UPTRIG(KEY_F)){
        decision->fric_mode = (decision->fric_mode == FRIC_ON) ? FRIC_OFF : FRIC_ON;
    }
    if(KEY_UPTRIG(KEY_E)){
        decision->mouse_fix = (decision->mouse_fix == MOUSE_FIX_ON) ? MOUSE_FIX_OFF : MOUSE_FIX_ON;
    }
    if(KEY_UPTRIG(KEY_Z)){
        if(decision->cam_target == CAM_TARGET_MID){
            decision->cam_target = CAM_TARGET_UP;
        } else if(decision->cam_target == CAM_TARGET_UP){
            decision->cam_target = CAM_TARGET_DOWN;
        } else {
            decision->cam_target = CAM_TARGET_MID;
        }
    }
    if(KEY_UPTRIG(KEY_Q) && (pDecisionAO->sniper == SNIPER_ON)){
        if(pDecisionAO->world_enable == WORLD_ENABLE_ON){
            WorldGimbalSetWorldAngles(
                _worldGimbal->WorldGimbalEstimate.world_yaw_deg,
                WORLD_GIMBAL_PRESET_PITCH_D);
        } else {
            GimbalSetPitchTarget(WORLD_GIMBAL_PRESET_PITCH_D);
        }
    }
    if(KEY_UPTRIG(KEY_R) && (pDecisionAO->sniper == SNIPER_ON)){
        upperComputerComm.Send.reserved[0] |= UPPER_PC_SHUTDOWN_REQUEST;
    }
    if(KEY_UPTRIG(KEY_B) && (pDecisionAO->sniper == SNIPER_ON)){
        uint8_t *const layout = &upperComputerComm.Send.reserved[1];

        *layout = (*layout >= CAMERA_LAYOUT_LAST) ? CAMERA_LAYOUT_FIRST : (uint8_t)(*layout + 1U);
    }

    if(pDecisionAO->sniper == SNIPER_OFF){
        distance_check.distance_check_translate.angle = 0;
    }
}

static void decision_handle_remote_input(uint8_t switch_left,
                                         uint8_t switch_right,
                                         uint8_t last_switch_left,
                                         uint8_t last_switch_right,
                                         float dial,
                                         float last_dial,
                                         uint8_t force_mode_event){
    DecisionAO *const decision = &DecisionAO_inst;
    uint8_t is_sniper = (switch_right == NORM_RC_SW_DOWN);
    uint8_t was_sniper = (last_switch_right == NORM_RC_SW_DOWN);
    uint8_t world_switch_edge =
        ((last_dial <= 0.1f) && (dial > 0.1f)) ||
        ((last_dial >= -0.1f) && (dial < -0.1f));

    if(VALUE_UPTRIG(switch_left == NORM_RC_SW_UP,
                    last_switch_left == NORM_RC_SW_UP) &&
       (switch_right != NORM_RC_SW_UP)){
        decision->fric_mode = (decision->fric_mode == FRIC_ON) ? FRIC_OFF : FRIC_ON;
    }
    if(((force_mode_event != 0U) && (is_sniper != 0U)) ||
       VALUE_UPTRIG(is_sniper, was_sniper) ||
       VALUE_DOWNTRIG(is_sniper, was_sniper)){
        post_decision_event(&event_switch_sniper);
    }
    if(world_switch_edge && (pDecisionAO->sniper == SNIPER_ON)){
        post_decision_event(&event_switch_world);
    }
}

static void decision_input_post(const NormRemoteCmd *command,
                                const NormRemoteCmd *last_command,
                                uint8_t force_terminal_event){
    uint8_t switch_left = command->Switch.switch_L1;
    uint8_t switch_right = command->Switch.switch_R1;
    uint8_t last_switch_left = last_command->Switch.switch_L1;
    uint8_t last_switch_right = last_command->Switch.switch_R1;
    uint8_t is_pc_mode;

    if((switch_left == NORM_RC_SW_UP) &&
       (switch_right == NORM_RC_SW_UP)){
        post_decision_event(&event_switch_protected);
        return;
    }

    is_pc_mode = (switch_left == NORM_RC_SW_MID) &&
                 (switch_right == NORM_RC_SW_UP);
    decision_post_terminal_event(is_pc_mode,
                                 switch_left,
                                 switch_right,
                                 last_switch_left,
                                 last_switch_right,
                                 force_terminal_event);

    if(is_pc_mode != 0U){
        decision_handle_pc_input(command, last_command);
    } else {
        decision_handle_remote_input(switch_left,
                                     switch_right,
                                     last_switch_left,
                                     last_switch_right,
                                     command->RelativeCH.ch4,
                                     last_command->RelativeCH.ch4,
                                     force_terminal_event);
    }
}

static void state_update(const NormRemoteCmd *command,
                         NormRemoteCmd *last_command){
    virtual_heat_update();

    switch(command->remote_source){
        case DT7:
            decision_input_post(command,
                                last_command,
                                last_command->remote_source != DT7);
            break;

        case VT13:
            break;

        case ERROR_RECEIVE:
        default:
            post_decision_event(&event_rc_lost);
            break;
    }

    memcpy(last_command, command, sizeof(*last_command));
}

static void virtual_heat_update(void){
    float cooling_per_period =
        (float)ext_game_robot_status.shooter_barrel_cooling_value /
        STATE_TASK_FREQUENCY_HZ;

    if((float)xvni > cooling_per_period){
        xvni = (uint16_t)((float)xvni - cooling_per_period);
    } else {
        xvni = 0U;
    }
}

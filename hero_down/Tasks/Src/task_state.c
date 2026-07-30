#include "task_state.h"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "decision_ao.h"
#include "general_define.h"
#include "judge_receive.h"
#include "task_receive.h"
#include "stirControl.h"
#include "task_monitor.h"

/*============================================================================
 * PC 键盘输入映射与边沿判定
 * 将当前帧与上一帧键盘状态封装为按键上升沿宏。
 *============================================================================*/
#define KEY_W PCKeyBoard.level_key_W
#define KEY_A PCKeyBoard.level_key_A
#define KEY_S PCKeyBoard.level_key_S
#define KEY_D PCKeyBoard.level_key_D
#define KEY_SHIFT PCKeyBoard.level_key_SHIFT
#define KEY_CTRL PCKeyBoard.level_key_CTRL
#define KEY_Q PCKeyBoard.level_key_Q
#define KEY_E PCKeyBoard.level_key_E
#define KEY_R PCKeyBoard.level_key_R
#define KEY_F PCKeyBoard.level_key_F
#define KEY_G PCKeyBoard.level_key_G
#define KEY_Z PCKeyBoard.level_key_Z
#define KEY_X PCKeyBoard.level_key_X
#define KEY_C PCKeyBoard.level_key_C
#define KEY_V PCKeyBoard.level_key_V
#define KEY_B PCKeyBoard.level_key_B

#define KeyVal(key) (command->key)
#define KeyValLast(key) (last_command->key)
#define KEY_UPTRIG(key) UPTRIG(KeyVal(key), KeyValLast(key))
#define KEY_DOWNTRIG(key) DOWNTRIG(KeyVal(key), KeyValLast(key))

/*============================================================================
 * 状态任务私有接口
 *============================================================================*/
static void state_update(const NormRemoteCmd* command,
                         NormRemoteCmd* last_command);
static void decision_input_post(const NormRemoteCmd* command,
                                const NormRemoteCmd* last_command);
static void post_decision_event(const QEvt* event);

/*============================================================================
 * 共享状态、输入历史与 QP 事件实例
 * `last_remote_command` 用于检测拨杆和按键边沿。
 *============================================================================*/
static RobotState robot_state;
const RobotState* const _robotState = &robot_state;

int crawler_rotate_flag;

static NormRemoteCmd last_remote_command;

static const QEvt event_switch_protected = {.sig = Switch_protected_SIG};
static const QEvt event_switch_pc = {.sig = Switch_PC_SIG};
static const QEvt event_switch_remote = {.sig = Switch_RC_SIG};
static const QEvt event_switch_sniper = {.sig = SWITCH_SNIPER_SIG};
static const QEvt event_switch_revolve = {.sig = SWITCH_REVOLVE_SIG};
static const QEvt event_switch_back = {.sig = SWITCH_BACK_SIG};
static const QEvt event_switch_stair = {.sig = SWITCH_STAIR_SIG};
static const QEvt event_rc_lost = {.sig = RC_LOST_SIG};

/*============================================================================
 * StateMachineTask — 遥控器输入状态任务
 * 每 10 ms 根据遥控器有效性、PC/RC 拨杆组合更新 DecisionAO。
 *============================================================================*/
/**
 * @brief   解析遥控输入并向决策主动对象投递状态事件
 * @param   argument 未使用
 * @retval  void
 */
void StateMachineTask(void* argument){
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

/*============================================================================
 * QP 事件投递与控制终端选择
 * 终端模式切换通过 SIG 交由 DecisionAO 状态机异步执行。
 *============================================================================*/
static void post_decision_event(const QEvt* event){
    QACTIVE_POST(AO_DecisionAO, event, NULL);
}

static void decision_post_terminal_event(uint8_t is_pc_mode,
                                         uint8_t switch_left,
                                         uint8_t switch_right,
                                         uint8_t last_switch_left,
                                         uint8_t last_switch_right){
    if((switch_right == last_switch_right) && (switch_left == last_switch_left)){
        return;
    }

    if(is_pc_mode != 0U){
        post_decision_event(&event_switch_pc);
    }
    else{
        post_decision_event(&event_switch_remote);
    }
}

/*============================================================================
 * PC 输入处理
 * 键盘触发主干状态 SIG，并直接更新摩擦轮、鼠标锁定和相机目标。
 *============================================================================*/
static void decision_handle_pc_input(const NormRemoteCmd* command,
                                     const NormRemoteCmd* last_command){
    DecisionAO* decision = &DecisionAO_inst;

    if(KEY_UPTRIG(KEY_X)){
        post_decision_event(&event_switch_sniper);
    }
    if(KEY_UPTRIG(KEY_Q)){
        post_decision_event(&event_switch_revolve);
    }
    if(KEY_UPTRIG(KEY_G)){
        post_decision_event(&event_switch_back);
    }
    if(KEY_UPTRIG(KEY_V)){
        post_decision_event(&event_switch_stair);
    }
    if(KEY_UPTRIG(KEY_F)){
        decision->fric_mode = (decision->fric_mode == FRIC_ON)
                              ? FRIC_OFF
                              : FRIC_ON;
    }
    if(KEY_UPTRIG(KEY_E)){
        decision->mouse_fix = (decision->mouse_fix == MOUSE_FIX_ON)
                              ? MOUSE_FIX_OFF
                              : MOUSE_FIX_ON;
    }
    if(KEY_UPTRIG(KEY_Z)){
        if(decision->cam_target == CAM_TARGET_MID){
            decision->cam_target = CAM_TARGET_UP;
        }
        else if(decision->cam_target == CAM_TARGET_UP){
            decision->cam_target = CAM_TARGET_DOWN;
        }
        else{
            decision->cam_target = CAM_TARGET_MID;
        }
    }
}

/*============================================================================
 * RC 输入处理
 * 左拨杆控制摩擦轮，右拨杆下档边沿切换狙击主干状态。
 *============================================================================*/
static void decision_handle_remote_input(uint8_t switch_left,
                                         uint8_t switch_right,
                                         uint8_t last_switch_left,
                                         uint8_t last_switch_right){
    DecisionAO* decision = &DecisionAO_inst;

    if(UPTRIG(switch_left == NORM_RC_SW_UP, last_switch_left == NORM_RC_SW_UP) && (switch_right != NORM_RC_SW_UP)){
        decision->fric_mode = (decision->fric_mode == FRIC_ON) ? FRIC_OFF : FRIC_ON;
    }
    if(UPTRIG(switch_right == NORM_RC_SW_DOWN, last_switch_right == NORM_RC_SW_DOWN) || DOWNTRIG(switch_right == NORM_RC_SW_DOWN, last_switch_right == NORM_RC_SW_DOWN)){
        post_decision_event(&event_switch_sniper);
    }
}

/*============================================================================
 * 拨弹输入处理
 * 按 PC/RC 模式写入拨弹目标，堵转检测最后覆盖为反转模式。
 *============================================================================*/
static void decision_handle_stir_input(const NormRemoteCmd* command,
                                       uint8_t is_pc_mode){
    DecisionAO* decision = &DecisionAO_inst;

    if(is_pc_mode != 0U){
        if(command->PCMouse.mouse_left
           && (decision->fric_mode == FRIC_ON)
           && ((ext_game_robot_status.shooter_barrel_heat_limit
                - ext_power_heat_data.shooter_42mm_barrel_heat) >= 100)){
            decision->stir_mode = STIR_ANGLE_CONTROL;
        }
    }
    else{
        if((command->Switch.switch_L1 == NORM_RC_SW_DOWN) && (decision->fric_mode != FRIC_OFF)){
            decision->stir_mode = STIR_ANGLE_CONTROL;
        }
        else if(_shootControl->ShootEstimate.stir_block_flag == 0U){
            decision->stir_mode = STIR_LOCK;
        }
    }

    if(_shootControl->ShootEstimate.stir_block_flag != 0U){
        decision->stir_mode = STIR_REVERSE;
    }
}

/*============================================================================
 * 遥控器输入路由
 * 保护组合优先；其余组合先确定 PC/RC，再处理对应来源的输入。
 *============================================================================*/
static void decision_input_post(const NormRemoteCmd* command,
                                const NormRemoteCmd* last_command){
    uint8_t switch_left = command->Switch.switch_L1;
    uint8_t switch_right = command->Switch.switch_R1;
    uint8_t last_switch_left = last_command->Switch.switch_L1;
    uint8_t last_switch_right = last_command->Switch.switch_R1;
    uint8_t is_pc_mode;

    if((switch_left == NORM_RC_SW_UP) && (switch_right == NORM_RC_SW_UP)){
        post_decision_event(&event_switch_protected);
        return;
    }

    is_pc_mode = (switch_left == NORM_RC_SW_MID) && (switch_right == NORM_RC_SW_UP);
    decision_post_terminal_event(is_pc_mode,
                                 switch_left,
                                 switch_right,
                                 last_switch_left,
                                 last_switch_right);

    if(is_pc_mode != 0U){
        decision_handle_pc_input(command, last_command);
    }
    else{
        decision_handle_remote_input(switch_left,
                                     switch_right,
                                     last_switch_left,
                                     last_switch_right);
    }
    decision_handle_stir_input(command, is_pc_mode);
}

/*============================================================================
 * 遥控器有效性与输入历史维护
 * DT7 数据驱动输入路由；接收异常投递失控事件并记录 RC_LOST。
 *============================================================================*/
static void state_update(const NormRemoteCmd* command,
                         NormRemoteCmd* last_command){
    switch(command->remote_source){
        case DT7:
            DecisionAO_inst.rc_lost_flag = RC_OK;
            decision_input_post(command, last_command);
            break;

        case VT13:
            break;

        case ERROR_RECEIVE:
        default:
            DecisionAO_inst.rc_lost_flag = RC_LOST;
            post_decision_event(&event_rc_lost);
            break;
    }

    memcpy(last_command, command, sizeof(*last_command));
}

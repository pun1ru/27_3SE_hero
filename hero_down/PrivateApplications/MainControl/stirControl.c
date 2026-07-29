#include "stirControl_internal.h"

#include <math.h>

#include "main.h"
#include "general_config_label.h"
#include "general_define.h"
#include "gimbalControl.h"
#include "judge_receive.h"
#include "peripheral_receive_task.h"
#include "state_task.h"
#include "decision_ao.h"
#include "DMJ4310.h"

static shoot_runtime_t g_shoot_runtime = {
    .selected_fire_mode = STIR_FIRE_TWO_STEP,
};
const ShootControl *const _shootControl = &g_shoot_runtime.control;

void ShootControlInitialize(void){
    stir_preset_angle_d = STIR_PRESET_ANGLE_D;
    stir_target_vol = STIR_MAX_SPEED_RPS;
    stir_enable_desire = DISABLE;
}

void ShootSetEnabled(uint8_t is_enabled){
    stir_enable_desire = (is_enabled != 0U) ? ENABLE : DISABLE;
}

void ShootVirtualHeatAddShot(void){
    if(virtual_heat < shoot_heat_limit){
        virtual_heat += SHOOT_VIRTUAL_HEAT_PER_SHOT;
    }
}

void ShootVirtualHeatSynchronize(uint16_t measured_heat){
    if(virtual_heat < measured_heat){
        virtual_heat = measured_heat;
    }
}

uint16_t ShootVirtualHeatGet(void){
    return virtual_heat;
}

uint16_t CRC16_Modbus(uint8_t *data, uint16_t length){
    uint16_t crc = CRC16_MODBUS_INITIAL_VALUE;

    for(uint16_t i = 0U; i < length; i++){
        crc ^= data[i]; // 与当前字节异或

        for(uint8_t j = 0U; j < CRC16_BITS_PER_BYTE; j++){
            if(crc & CRC16_MODBUS_LOW_BIT_MASK){
                crc >>= CRC16_SHIFT_BITS;
                crc ^= CRC16_MODBUS_POLYNOMIAL;
            } else {
                crc >>= CRC16_SHIFT_BITS;
            }
        }
    }

    return crc;
}
static uint8_t shoot_update_stall_recovery(void){
    if(stir_block_flag == ENABLE && last_stir_block == DISABLE &&
       stall_recovery_state == STALL_RECOVERY_IDLE){
        two_step_phase = STIR_FEED_IDLE;
        stir_active_flag = DISABLE;
        stir_delay_cycles = 0U;
        stall_preset_target_d = stir_all_target_pos_d;
        stall_reverse_target_d = stir_all_angle_d + STIR_REVERSE_ANGLE_D;
        stir_all_target_pos_d = stall_reverse_target_d;
        stall_count = 0U;
        stir_block_flag = DISABLE;
        stall_recovery_state = STALL_RECOVERY_REVERSING;
    }
    last_stir_block = stir_block_flag;

    switch(stall_recovery_state){
        case STALL_RECOVERY_REVERSING:
            if(stir_block_flag == ENABLE){
                stall_count = 0U;
                stir_block_flag = DISABLE;
                stir_all_target_pos_d = stall_preset_target_d;
                stall_recovery_state = STALL_RECOVERY_RETURNING;
            } else if(fabsf(stir_all_angle_d - stall_reverse_target_d) <
                      STIR_RECOVERY_TOLERANCE_D){
                stir_all_target_pos_d = stall_preset_target_d;
                stall_recovery_state = STALL_RECOVERY_RETURNING;
            }
            break;

        case STALL_RECOVERY_RETURNING:
            if(stir_block_flag == ENABLE){
                stall_count = 0U;
                stir_block_flag = DISABLE;
                stall_recovery_state = STALL_RECOVERY_IDLE;
                StirTargetAngleSet();
            } else if(fabsf(stir_all_angle_d - stall_preset_target_d) < STIR_RECOVERY_TOLERANCE_D){
                stall_recovery_state = STALL_RECOVERY_IDLE;
                StirTargetAngleSet();
            }
            break;

        case STALL_RECOVERY_IDLE:
        default:
            break;
    }

    return (stall_recovery_state != STALL_RECOVERY_IDLE);
}

static void shoot_update_fire_mode(uint8_t in_stall_recovery){
    if(pDecisionAO->sniper != SNIPER_ON){
        fire_mode = STIR_FIRE_SINGLE_STEP;
    } else if(stir_block_flag == DISABLE && !in_stall_recovery){
        if(rc_ch0 > STIR_FIRE_MODE_CH0_THRESHOLD){
            fire_mode = STIR_FIRE_TWO_STEP;
        } else if(rc_ch0 < -STIR_FIRE_MODE_CH0_THRESHOLD){
            fire_mode = STIR_FIRE_SINGLE_STEP;
        }
    }
}

static void shoot_update_feed_state(uint8_t in_stall_recovery){
    switch(two_step_phase){
        case STIR_FEED_IDLE:
            if(pDecisionAO->stir_mode == STIR_ANGLE_CONTROL &&
               fabsf(stir_all_target_pos_d - stir_all_angle_d) < STIR_FEED_TOLERANCE_D &&
               stir_block_flag == DISABLE && !in_stall_recovery){
                if(fire_mode == STIR_FIRE_SINGLE_STEP){
                    stir_all_target_pos_d -= STIR_SINGLE_STEP_ANGLE_D;
                    two_step_phase = STIR_FEED_SECOND_STEP;
                } else {
                    stir_all_target_pos_d -= STIR_TWO_STEP_FIRST_ANGLE_D;
                    two_step_phase = STIR_FEED_FIRST_STEP;
                }
                stir_delay_cycles = 0U;
                stir_active_flag = ENABLE;
                shoot_count++;
                ShootVirtualHeatAddShot();
                stall_count = 0U;
                stir_block_flag = DISABLE;
            }
            break;

        case STIR_FEED_FIRST_STEP:
            if(fabsf(stir_all_target_pos_d - stir_all_angle_d) < STIR_FEED_TOLERANCE_D){
                if(++stir_delay_cycles >= STIR_TWO_STEP_DELAY_CYCLES){
                    stir_all_target_pos_d -= STIR_TWO_STEP_SECOND_ANGLE_D;
                    two_step_phase = STIR_FEED_SECOND_STEP;
                    stir_delay_cycles = 0U;
                }
            } else {
                stir_delay_cycles = 0U;
            }
            break;

        case STIR_FEED_SECOND_STEP:
            if(pDecisionAO->stir_mode == STIR_LOCK){
                two_step_phase = STIR_FEED_IDLE;
                stir_active_flag = DISABLE;
            }
            break;
    }
}

static void shoot_update_input_target(void){
    stir_target_pos_rad = stir_target_pos / STIR_DEGREES_PER_RADIAN * PI;
    stir_all_target_pos_rad = stir_all_target_pos_d / STIR_DEGREES_PER_RADIAN * PI;
    shoot_flag = (pDecisionAO->stir_mode != STIR_LOCK) ? ENABLE : DISABLE;
}

static void shoot_update_stir_motor_state(uint8_t in_stall_recovery){
    if(pDecisionAO->ctrl_terminal == CONTROL_STOP){
        lock_motor(&hfdcan1, GMJ4310MOTOR_ID);
    } else if(in_stall_recovery){
        if(stir_motor_state == STIR_MOTOR_DISABLED){
            start_motor(&hfdcan1, GMJ4310MOTOR_ID);
        }
    } else if(stir_block_flag == ENABLE && stir_motor_state == STIR_MOTOR_ENABLED){
        lock_motor(&hfdcan1, GMJ4310MOTOR_ID);
    } else if(pDecisionAO->ctrl_terminal != CONTROL_STOP && stir_block_flag == DISABLE &&
              stir_motor_state == STIR_MOTOR_DISABLED){
        start_motor(&hfdcan1, GMJ4310MOTOR_ID);
    }
}

void ShootInputUpdate(void){
    uint8_t in_stall_recovery;

    in_stall_recovery = shoot_update_stall_recovery();
    shoot_update_fire_mode(in_stall_recovery);
    shoot_update_feed_state(in_stall_recovery);
    shoot_update_input_target();
    shoot_update_stir_motor_state(in_stall_recovery);
    GetStirRealAngle();
}

static void shoot_update_stall_detection(void){
    if(fabsf(stir_motor_speed_radps) < STIR_CAUTION_SPEED_RADPS &&
       fabsf(stir_motor_torque_nm) > STIR_STALL_TORQUE_THRESHOLD_NM){
        stall_count += STIR_STALL_COUNTER_STEP;
    } else if(stall_count > 0U){
        stall_count--;
    }

    if(stall_count >= STIR_STALL_TRIGGER_COUNT){
        stir_block_flag = ENABLE;
    }
    if(stall_count == 0U){
        stir_block_flag = DISABLE;
    }
}

static void shoot_update_start_edge(void){
    if(last_robot_state == CONTROL_STOP && pDecisionAO->ctrl_terminal != CONTROL_STOP &&
       stir_motor_frame_count > 0U){
        StirTargetAngleSet();
    }
    last_robot_state = pDecisionAO->ctrl_terminal;
}

static void shoot_update_calibration(void){
    if(stir_motor_speed_radps < STIR_CAUTION_SPEED_RADPS &&
       pDecisionAO->ctrl_terminal != CONTROL_STOP &&
       !stir_reset_flag){
        calibration_cycles++;
        if(calibration_cycles % STIR_CALIBRATION_PERIOD_CYCLES == 0U){
            stir_real_angle = stir_target_pos;
        }
    } else {
        calibration_cycles = 0U;
    }
}

static void shoot_update_offline_state(void){
    uint16_t current_frame_count = stir_motor_frame_count;

    frame_check_cycles++;
    if(frame_check_cycles % STIR_OFFLINE_CHECK_PERIOD_CYCLES == 0U){
        if(previous_frame_count == last_frame_count && last_frame_count == current_frame_count &&
           !stir_reset_flag){
            stir_real_angle = 0.0f;
            stir_real_angle_d = 0.0f;
            stir_angle_last_d = 0.0f;
            stir_angle_cur_d = 0.0f;
            stir_real_angle_rad = 0.0f;
            stir_reset_flag = ENABLE;
        }
        last_frame_count = current_frame_count;
        previous_frame_count = last_frame_count;
    }

    GetStirRealAngle();
    if(!stir_reset_flag && stir_motor_frame_count > 0U){
        GetStirRealAngle();
    }
    if(stir_reset_flag && shooter_output_enabled && current_frame_count != last_frame_count){
        stir_reset_flag = DISABLE;
        GetStirRealAngle();
        StirTargetAngleSet();
    }
}

void ShootEstimateUpdate(void){
    shoot_update_stall_detection();
    shoot_update_start_edge();
    shoot_update_calibration();
    shoot_update_offline_state();
}
void GetStirRealAngle(void){
    stir_angle_cur_d = stir_motor_position_d;
    stir_real_angle_d = stir_angle_cur_d - stir_angle_last_d;
    stir_real_angle += stir_real_angle_d;

    if(stir_angle_cur_d < -DM_MOTO_MAX_ENCODE_D + STIR_ENCODER_WRAP_MARGIN_D &&
       stir_angle_last_d > DM_MOTO_MAX_ENCODE_D - STIR_ENCODER_WRAP_MARGIN_D){
        stir_turn_count++;
    }
    if(stir_angle_cur_d > DM_MOTO_MAX_ENCODE_D - STIR_ENCODER_WRAP_MARGIN_D &&
       stir_angle_last_d < -DM_MOTO_MAX_ENCODE_D + STIR_ENCODER_WRAP_MARGIN_D){
        stir_turn_count--;
    }
    stir_all_angle_d = stir_motor_position_d + stir_turn_count * DM_MOTO_MAX_ENCODE_D;

    stir_angle_last_d = stir_angle_cur_d;
    stir_real_angle_rad = stir_real_angle / STIR_DEGREES_PER_RADIAN * PI;
}

void StirTargetAngleSet(void){
    float relative_angle_d = stir_motor_position_d - stir_preset_angle_d;
    float delta_angle_d = fmodf(relative_angle_d, STIR_CHAMBER_SPACING_D);

    if(delta_angle_d > STIR_FORWARD_SELECTION_D){
        delta_angle_d -= STIR_CHAMBER_SPACING_D;
    }
    if(delta_angle_d < -(STIR_CHAMBER_SPACING_D - STIR_FORWARD_SELECTION_D)){
        delta_angle_d += STIR_CHAMBER_SPACING_D;
    }

    stir_all_target_pos_d =
        stir_motor_position_d - delta_angle_d + stir_turn_count * DM_MOTO_MAX_ENCODE_D;
    stir_all_target_pos_rad = stir_all_target_pos_d / STIR_DEGREES_PER_RADIAN * PI;
}

void ShootControlUpdate(void){
    if(stir_block_flag && stall_recovery_state == STALL_RECOVERY_IDLE){
        stir_target_vol = STIR_STOP_SPEED_RPS;
    } else {
        if(shoot_gimbal_pitch_d < STIR_LOW_SPEED_PITCH_THRESHOLD_D){
            stir_target_vol = STIR_MAX_SPEED_LOW_PITCH_RPS;
        } else {
            stir_target_vol = STIR_MAX_SPEED_RPS;
        }
    }
    if(CONTROL_STOP == pDecisionAO->ctrl_terminal){
        stir_enable_desire = DISABLE;
    }

#if defined SHOOT_OFF
    fric_target_output[LEFT] = fric_target_output[RIGHT] = SHOOT_MOTOR_OUTPUT_ZERO;
    stir_enable_desire = DISABLE;
#endif
}

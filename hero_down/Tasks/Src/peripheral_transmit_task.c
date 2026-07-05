#include "tim.h"
#include "usart.h"
#include "general_task_include.h"
#include "LK_driver.h"
#include "bsp_dwt.h"
#include "dm_imu.h"
#include "LK_485_driver.h"

/* ==================================================================
 * HAL 回调
 * ================================================================== */
uint8_t uart10_tx_complete = 1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == PITCH_UART.Instance)
        uart10_tx_complete = 1;
}

/* ==================================================================
 * MotorControlCANSend — CAN 总线电机控制帧发送 + 双板通信 UART
 * ControlTask 每个周期调用一次（1kHz，内部分频 7 时隙）
 * ================================================================== */
extern JointControl        jointControl;
extern GimbalControl       gimbalControl;
extern const NormRemoteCmd* _normRemoteCmd;
extern int                 crawler_rotate_flag;

/* ---- 双板通信帧 ---- */
#define MCU_FRAME_LEN  (1U + 21U + 4U + 2U + 12U)
extern uint8_t  dt7RecBuffer[18U];
extern uint8_t  VT3RecBuffer[21U];
extern DMJ4310MotorRec DMyawMotorRec;
extern float    yaw_dm_forward_offset_rad;
extern JointBodyState g_joint_body_state_body_dbg;
uint8_t double_mcu_frame[MCU_FRAME_LEN];

void MotorControlCANSend(void)
{
    static uint8_t slot = 0;

    /* CAN 总线电机维护 ID 数组 */
    static const uint16_t can1_maintain[] = {
        CAN1_JOINT_LF, CAN1_JOINT_RF, CAN1_JOINT_RB, CAN1_JOINT_LB,
        CAN1_YAW, CAN1_STIR
    };
    static const uint16_t can2_maintain[] = {
        CAN2_CATERPILLAR_L, CAN2_CATERPILLAR_R
    };

    /* ====== 停止态：锁电机 + 清错误 + 零值 ====== */
    if (_robotState->ctrl_terminal == CONTROL_STOP)
    {
        switch (slot)
        {
        case 0:
            DM_MITControl_JointsSendTorq(&hfdcan1, (float[4]){0,0,0,0});
            break;
        case 1:
            DM_MITControl_Send(&hfdcan1, CAN1_YAW, 0,0,0,0,0);
            break;
        case 2:
            DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_L, 0,0,0,0,0);
            DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_R, 0,0,0,0,0);
            break;
        case 3:
            CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
            break;
        case 4:
            CANTransmit_I16(&hfdcan2, 0x2FF, 0,0,
                ext_game_robot_status.chassis_power_limit - 1, 0);
            break;
        case 5:
            Motors_Lock(&hfdcan1, can1_maintain, CAN1_MAINTAIN_COUNT);
            Motors_ClearError(&hfdcan1, can1_maintain, CAN1_MAINTAIN_COUNT);
            ctrl_motor2(&hfdcan1, GMJ4310MOTOR_ID, 0, 0);
            break;
        case 6:
            Motors_Lock(&hfdcan2, can2_maintain, CAN2_MAINTAIN_COUNT);
            Motors_ClearError(&hfdcan2, can2_maintain, CAN2_MAINTAIN_COUNT);
            break;
        }
        slot = (slot + 1) % 7;
        return;
    }

    /* ====== 正常运行态 ====== */
    switch (slot)
    {
    case 0:
        DM_MITControl_JointsSendTorq(&hfdcan1,
            jointControl.JointMotorControl.mit_Tff);
        break;
    case 1:
        DM_MITControl_Send(&hfdcan1, CAN1_YAW,
            gimbalControl.GimbalMotorControl.mit.p,
            gimbalControl.GimbalMotorControl.mit.v,
            gimbalControl.GimbalMotorControl.mit.Kp,
            gimbalControl.GimbalMotorControl.mit.Kd,
            gimbalControl.GimbalMotorControl.mit.Tff);
        break;
    case 2:
        DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_L,
            0,0,0,0, -3.1f * crawler_rotate_flag);
        DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_R,
            0,0,0,0,  3.1f * crawler_rotate_flag);
        break;
    case 3:
        CANTransmit_I16(&hfdcan2, 0x200,
            _chassisControl->WheelMotorControl.target_motor_output[0],
            _chassisControl->WheelMotorControl.target_motor_output[1],
            _chassisControl->WheelMotorControl.target_motor_output[2],
            _chassisControl->WheelMotorControl.target_motor_output[3]);
        break;
    case 4:
        CANTransmit_I16(&hfdcan2, 0x2FF, 0, 0,
            ext_game_robot_status.chassis_power_limit - 1, 0);
        break;
    case 5:
        Motors_Start(&hfdcan1, can1_maintain, CAN1_MAINTAIN_COUNT);
        ctrl_motor2(&hfdcan1, GMJ4310MOTOR_ID,
            _shootControl->ShootTargetInput.stir_all_target_pos_rad,
            _shootControl->ShootTargetInput.stir_target_vol);
        break;
    case 6:
        Motors_Start(&hfdcan2, can2_maintain, CAN2_MAINTAIN_COUNT);
        break;
    }

    slot = (slot + 1) % 7;

    /* ---- 双板通信：UART 转发遥控器 + yaw + HP + 机体姿态到上板 ---- */
    HAL_GPIO_WritePin(RS485_MASTER_DE_GPIO_Port, RS485_MASTER_DE_Pin, GPIO_PIN_SET);
    if (_normRemoteCmd->remote_source == DT7)
    {
        double_mcu_frame[0] = 0x07;
        memcpy(double_mcu_frame + 1, dt7RecBuffer, 18U);
    }
    else if (_normRemoteCmd->remote_source == VT13)
    {
        double_mcu_frame[0] = 0x03;
        memcpy(double_mcu_frame + 1, VT3RecBuffer, 21U);
    }
    else
    {
        double_mcu_frame[0] = 0x88;
        double_mcu_frame[1] = 0x88;
    }
    float yaw_enc_deg_tx = AngleLimit((DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f, -180.0f, 180.0f);
    memcpy(double_mcu_frame + 1U + 21U,                              &yaw_enc_deg_tx, sizeof(yaw_enc_deg_tx));
    memcpy(double_mcu_frame + 1U + 21U + 4U,                         &ext_game_robot_status.current_HP, sizeof(ext_game_robot_status.current_HP));
    memcpy(double_mcu_frame + 1U + 21U + 4U + 2U,                    &g_joint_body_state_body_dbg.pitch_d, sizeof(float));
    memcpy(double_mcu_frame + 1U + 21U + 4U + 2U + 4U,               &g_joint_body_state_body_dbg.roll_d,  sizeof(float));
    memcpy(double_mcu_frame + 1U + 21U + 4U + 2U + 8U,               &g_joint_body_state_body_dbg.yaw_d,   sizeof(float));
    HAL_UART_Transmit_DMA(&MASTER_485_UART, double_mcu_frame, MCU_FRAME_LEN);
}

/* ==================================================================
 * DebugTask — 调试数据发送（yaw ADRC 观测值）
 * ================================================================== */

/* ---- Debug 外部引用 ---- */
extern const GimbalControl* _gimbalControl;
extern DMJ4310MotorRec      stirMotorRec;

/* ---- Debug 本地数据 ---- */
static uint8_t debug_data[42];

/* ---- Debug 工具函数 ---- */
static uint8_t crc8_maxim(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static void DebugTransmit(void)
{
#if 0
    /* ===== 原始帧格式（保留） ===== */
    static uint16_t debug_seq = 0;
    /* 帧格式: 帧头 2B | 序号 2B | shoot_flag 1B | initial_speed 4B |
               fric_rpm[6] 12B | pitch_angle 4B | yaw_angle 4B |
               stir_torque 4B | stir_speed 4B | CRC-8 1B = 38B */
    debug_data[0] = 0xAA;
    debug_data[1] = 0xBB;

    memcpy(&debug_data[2], &debug_seq, 2);
    debug_seq++;

    debug_data[4] = (uint8_t)_shootControl->ShootTargetInput.shoot_flag;

    memcpy(&debug_data[5], &ext_shoot_data.initial_speed, 4);

    memcpy(&debug_data[9],  (void*)&gimbal_fric_rpm_rx_arr[0], 2);
    memcpy(&debug_data[11], (void*)&gimbal_fric_rpm_rx_arr[1], 2);
    memcpy(&debug_data[13], (void*)&gimbal_fric_rpm_rx_arr[2], 2);
    memcpy(&debug_data[15], (void*)&gimbal_fric_rpm_rx_arr[3], 2);
    memcpy(&debug_data[17], (void*)&gimbal_fric_rpm_rx_arr[4], 2);
    memcpy(&debug_data[19], (void*)&gimbal_fric_rpm_rx_arr[5], 2);

    memcpy(&debug_data[21], (void*)&gimbal_pitch_rx_d, 4);

    memcpy(&debug_data[25], (void*)&_gimbalControl->GimbalEstimate.yaw_angle_d, 4);

    memcpy(&debug_data[29], &stirMotorRec.toq, 4);

    memcpy(&debug_data[33], &stirMotorRec.vel_radps, 4);

    debug_data[37] = crc8_maxim(debug_data, 37);

    HAL_UART_Transmit_DMA(&huart7, debug_data, 38);

#else
    /* ===== yaw ADRC 调试帧 =====
     * TD.x1  TD跟踪位置(度)
     * ESO.z1 ESO估计位置(度)
     * ESO.z3 ESO扰动估计
     * ADRC.u LADRC控制输出
     * yaw_target yaw目标角(度)
     * yaw_raw    yaw原始反馈(度)
     */
    debug_data[0] = 0xAA;
    debug_data[1] = 0xBB;

    float td_x1_deg  = gimbalControl.GimbalMotorControl.yaw_ADRC.td.x1  * (180.0f / 3.141592f);
    float eso_z1_deg = gimbalControl.GimbalMotorControl.yaw_ADRC.eso.z1 * (180.0f / 3.141592f);
    memcpy(&debug_data[2],  (void*)&td_x1_deg, 4);
    memcpy(&debug_data[6],  (void*)&eso_z1_deg, 4);
    memcpy(&debug_data[10], (void*)&gimbalControl.GimbalMotorControl.yaw_ADRC.eso.z3, 4);
    memcpy(&debug_data[14], (void*)&gimbalControl.GimbalMotorControl.yaw_ADRC.u, 4);
    memcpy(&debug_data[18], (void*)&gimbalControl.GimbalTargetInput.yaw_angle_d, 4);
    memcpy(&debug_data[22], (void*)&_gimbalControl->GimbalEstimate.yaw_angle_d, 4);

    HAL_UART_Transmit_DMA(&huart7, debug_data, 26);
#endif
}

void DebugTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._debug_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._debug_task = &this_tick_count;

    current_tick_count = last_tick_count = xTaskGetTickCount();
    while (1)
    {
#ifdef DEBUG_MSG_ENABLE
        DebugTransmit();
#endif
        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
        vTaskDelayUntil(&current_tick_count, DEBUG_TASK_PERIOD_SET);
    }
}

/* ==================================================================
 * UIOperationTask — 裁判系统 UI 绘制
 * 绘制实现位于 UI_design 模块
 * ================================================================== */
void UIOperationTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._ui_operation_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._ui_operation_task = &this_tick_count;

    current_tick_count = last_tick_count = xTaskGetTickCount();
    while (1)
    {
        /* 降低发送频率至 ~27.8Hz（3ms×12=36ms），满足裁判系统0x0301的30Hz上限 */
        if (task_counter % 12 == 0)
            UiOperation();

        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
        vTaskDelayUntil(&current_tick_count, UI_OPERATION_TASK_PERIOD_SET);
    }
}
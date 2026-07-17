#include "tim.h"
#include "usart.h"
#include "general_task_include.h"
#include "LK_driver.h"
#include "bsp_dwt.h"
#include "dm_imu.h"
#include "LK_485_driver.h"
#include "../../PrivateDrivers/Board2Borad/Board2Board.h"
#include "../../PrivateApplications/System_IDF/system_idf.h"
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
 *
 * ControlTask 每个周期调用一次（500Hz）
 * 20-slot 多级分频，Yaw 每帧都发：
 *   T1 ( 500Hz): Yaw MIT
 *   T2 ( 250Hz): 关节 MIT ×4,   轮电机 0x200
 *   T2 ( 100Hz): 拨弹 Stir
 *   T3 ( 125Hz): 履带 MIT ×2
 *   T3 ( 100Hz): 超级电容 0x2FF
 *   T4 ( ~31Hz): 维护 Start/ClearError（1帧/slot 散布，16帧完整周期）
 *
 * 带宽预算：FDCAN1=1Mbps(≤7帧/ms), FDCAN2=781Kbps(≤5帧/ms)
 *   FDCAN1 峰值 6帧/slot(40%), FDCAN2 峰值 4帧/slot(24%)
 * ================================================================== */
extern JointControl        jointControl;
extern GimbalControl       gimbalControl;
extern const NormRemoteCmd* _normRemoteCmd;
extern int                 crawler_rotate_flag;
/* ---- 双板通信帧 ---- */
extern DMJ4310MotorRec DMyawMotorRec;
extern float    yaw_dm_forward_offset_rad;
extern JointBodyState g_joint_body_state_body_dbg;
void MotorControlCANSend(void)
{
    static uint8_t slot = 0;   /* 20-slot 循环, 20ms 周期 */
    /* CAN 总线电机维护 ID 数组 */
    static const uint16_t can1_maintain[] = {
        CAN1_JOINT_LF, CAN1_JOINT_RF, CAN1_JOINT_RB, CAN1_JOINT_LB,
        CAN1_YAW, CAN1_STIR
    };
    static const uint16_t can2_maintain[] = {
        CAN2_CATERPILLAR_L, CAN2_CATERPILLAR_R
    };
    /* ====== 停止态：锁电机 + 清错误 + 零值 ====== */
    if (pDecisionAO->ctrl_terminal == CONTROL_STOP
        || pDecisionAO->can_enable == CAN_DISABLE)
    {
        /* T1: Yaw 零值 — 每 slot */
        DM_MITControl_Send(&hfdcan1, CAN1_YAW, 0,0,0,0,0);
        /* T2: 关节零值 + 轮零值 — 偶数 slot */
        if ((slot & 1) == 0)
        {
            DM_MITControl_JointsSendTorq(&hfdcan1, (float[4]){0,0,0,0});
            CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
        }
        /* T2: 拨弹零值 — 每5 slot */
        if (slot % 5 == 0)
            ctrl_motor2(&hfdcan1, GMJ4310MOTOR_ID, 0, 0);
        /* T3: 履带零值 — 每4 slot 偏移1 */
        if (slot % 4 == 1)
        {
            DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_L, 0,0,0,0,0);
            DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_R, 0,0,0,0,0);
        }
        /* T3: 功率限制 — 每5 slot 偏移3 */
        if (slot % 5 == 3)
            CANTransmit_I16(&hfdcan2, 0x2FF, 0,0,
                ext_game_robot_status.chassis_power_limit - 1, 0);
        /* T4: 维护 — 1帧/slot 散布（Lock + ClearError 交替） */
        {
            static uint8_t maint_seq_stop = 0;
            if (maint_seq_stop < 12)
            {
                /* CAN1: 先 Lock 6个, 再 ClearError 6个 */
                if (maint_seq_stop < 6)
                    lock_motor(&hfdcan1, can1_maintain[maint_seq_stop]);
                else
                    clear_error(&hfdcan1, can1_maintain[maint_seq_stop - 6]);
            }
            else
            {
                /* CAN2: 先 Lock 2个, 再 ClearError 2个 */
                uint8_t i = maint_seq_stop - 12;
                if (i < 2)
                    lock_motor(&hfdcan2, can2_maintain[i]);
                else if (i < 4)
                    clear_error(&hfdcan2, can2_maintain[i - 2]);
            }
            maint_seq_stop = (maint_seq_stop + 1) % 16;
        }
        slot = (slot + 1) % 20;
        /* 不 return，继续执行下面的双板通信 RS485 发送 */
    }
    else
    {
    /* ====== 正常运行态 ====== */
#define ZERO_JOINTS
#define ZERO_WHEELS
//#define ZERO_STIR
//#define ZERO_YAW
#define ZERO_CATERPILLAR
    /* ---- T1: Yaw — 每 slot (500Hz) ---- */
#ifdef ZERO_YAW
    DM_MITControl_Send(&hfdcan1, CAN1_YAW, 0,0,0,0,0);
#else
    DM_MITControl_Send(&hfdcan1, CAN1_YAW,
        gimbalControl.GimbalMotorControl.mit.p,
        gimbalControl.GimbalMotorControl.mit.v,
        gimbalControl.GimbalMotorControl.mit.Kp,
        gimbalControl.GimbalMotorControl.mit.Kd,
        gimbalControl.GimbalMotorControl.mit.Tff);
#endif
    /* ---- T2: 关节 MIT ×4 — 偶数 slot (250Hz) ---- */
    if ((slot & 1) == 0)
    {
        static const uint16_t jids[4] = {CAN1_JOINT_LF, CAN1_JOINT_RF, CAN1_JOINT_RB, CAN1_JOINT_LB};
        for (int i = 0; i < 4; i++)
#ifdef ZERO_JOINTS
            DM_MITControl_Send(&hfdcan1, jids[i], 0, 0, 0, 0, 0);
#else
            DM_MITControl_Send(&hfdcan1, jids[i],
                jointControl.JointMotorControl.mit_p[i],
                jointControl.JointMotorControl.mit_v[i],
                jointControl.JointMotorControl.mit_Kp[i],
                jointControl.JointMotorControl.mit_Kd[i],
                jointControl.JointMotorControl.mit_Tff[i]);
#endif
    }
    /* ---- T2: 轮电机 0x200 — 偶数 slot (250Hz) ---- */
    if ((slot & 1) == 0)
#ifdef ZERO_WHEELS
        CANTransmit_I16(&hfdcan2, 0x200, 0,0,0,0);
#else
        CANTransmit_I16(&hfdcan2, 0x200,
            _chassisControl->WheelMotorControl.target_motor_output[0],
            _chassisControl->WheelMotorControl.target_motor_output[1],
            _chassisControl->WheelMotorControl.target_motor_output[2],
            _chassisControl->WheelMotorControl.target_motor_output[3]);
#endif
    /* ---- T2: 拨弹 Stir — 每5 slot (100Hz) ---- */
    if (slot % 5 == 0)
#ifdef ZERO_STIR
        ctrl_motor2(&hfdcan1, GMJ4310MOTOR_ID, 0, 0);
#else
        ctrl_motor2(&hfdcan1, GMJ4310MOTOR_ID,
            _shootControl->ShootTargetInput.stir_all_target_pos_rad,
            _shootControl->ShootTargetInput.stir_target_vol);
#endif
    /* ---- T3: 履带 MIT ×2 — 每4 slot 偏移1 (125Hz) ---- */
    if (slot % 4 == 1)
#ifdef ZERO_CATERPILLAR
    {
        DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_L, 0,0,0,0,0);
        DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_R, 0,0,0,0,0);
    }
#else
    {
        DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_L,
            0,0,0,0, -3.1f * crawler_rotate_flag);
        DM_MITControl_Send(&hfdcan2, CAN2_CATERPILLAR_R,
            0,0,0,0,  3.1f * crawler_rotate_flag);
    }
#endif
    /* ---- T3: 超级电容功率限制 0x2FF — 每5 slot 偏移3 (100Hz) ---- */
    if (slot % 5 == 3)
        CANTransmit_I16(&hfdcan2, 0x2FF, 0, 0,
            ext_game_robot_status.chassis_power_limit - 1, 0);
    /* ---- T4: 维护 Start — 1帧/slot 散布 (~42Hz 完整周期) ---- */
    {
        static uint8_t maint_seq = 0;
        if (maint_seq < 6)
            start_motor(&hfdcan1, can1_maintain[maint_seq]);
        else
            start_motor(&hfdcan2, can2_maintain[maint_seq - 6]);
        maint_seq = (maint_seq + 1) % 8;
    }
    slot = (slot + 1) % 20;
    }  /* else: 正常运行态结束 */
    /* ---- 双板通信 CAN：替代 RS485 转发 ---- */
    {
        float yaw_enc = AngleLimit((DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f, -180.0f, 180.0f);
        B2BSendGimbalInput();   /* 0x221 500Hz 云台pitch控制（摇杆+鼠标） */
        B2BSendBodyState(g_joint_body_state_body_dbg.roll_d,
                         g_joint_body_state_body_dbg.pitch_d,
                         g_joint_body_state_body_dbg.yaw_d,
                         yaw_enc);                        /* 0x220 500Hz 机体姿态+yaw编码器 */
        if (slot % 5 == 0)
            B2BSendKeysSwitch();                           /* 0x222 100Hz 键位+开关+HP */
    }
}
/* ==================================================================
 * DebugTask — 调试数据发送（yaw ADRC 观测值）
 * ================================================================== */
/* ---- Debug 外部引用 ---- */
extern const GimbalControl* _gimbalControl;
extern DMJ4310MotorRec      stirMotorRec;
extern volatile int16_t     gimbal_fric_rpm_rx_arr[6];
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
/* ---- Debug 帧类型选择（只定义一个） ---- */
#define DEBUG_FRAME_CTRL     /* ADRC调试帧 0xAABB 34B */
//#define DEBUG_FRAME_SYSID      /* 系统辨识调试帧 0xAABB 30B */
static void DebugTransmit(void)
{
    /* ---- 公用外部引用 ---- */
    extern DMJ4310MotorRec DMyawMotorRec;
    extern volatile float gimbal_yaw_dps_rx;
    extern volatile float gimbal_yaw_rx_d;
    extern float yaw_dm_forward_offset_rad;
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
#elif defined(DEBUG_FRAME_CTRL)
	    /* ===== ADRC调试帧 (42B) =====
     * [0-1]   0xAA 0xBB  帧头
     * [2-5]   float       td_x1_deg       TD跟踪位置 (deg)
     * [6-9]   float       adrc_u          ADRC最终控制量u
     * [10-13] float       eso_z3          ESO扰动估计z3
     * [14-17] float       eso_z2          ESO速度估计z2 (deg/s)
     * [18-21] float       esf_e1_deg      位置误差e1 (deg)
     * [22-25] float       eso_z1_deg      ESO位置估计z1 (deg)
     * [26-29] float       yaw_actual_deg  估计yaw角度 (deg, GimbalEstimate)
	     * [30-33] float       yaw_actual_dps  实际yaw角速度 (deg/s, 上板IMU)
	     * [34-37] float       yaw_imu_raw_d   上板传下IMU原始yaw角度 (deg)
	     * [38-41] float       yaw_enc_raw_d   yaw编码器原始角度 (deg, 未滤波)
	     */
    extern const GimbalControl* _gimbalControl;
    const ADRC* adrc = &_gimbalControl->GimbalMotorControl.yaw_ADRC;
    float td_x1_d    = adrc->td.x1 * 57.29578f;
    float adrc_u     = adrc->u;
    float eso_z3     = adrc->eso.z3;
    float eso_z2     = adrc->eso.z2 * 57.29578f;   /* rad/s→deg/s */
    float esf_e1_d   = adrc->esf.e_1 * 57.29578f;
    float eso_z1_d   = adrc->eso.z1 * 57.29578f;
    float yaw_actual = _gimbalControl->GimbalEstimate.yaw_angle_d;
    float yaw_dps    = gimbal_yaw_dps_rx;         /* IMU原始角速度, 不用estimate */
    float yaw_imu_d  = gimbal_yaw_rx_d;          /* 上板传下来的IMU原始yaw角度 */
    float yaw_enc_raw_d = (DMyawMotorRec.pos_d - yaw_dm_forward_offset_rad) * 57.29578f; /* 编码器原始角度(deg) */

    debug_data[0] = 0xAA;
    debug_data[1] = 0xBB;
    memcpy(&debug_data[2],  (void*)&td_x1_d,      4);
    memcpy(&debug_data[6],  (void*)&adrc_u,       4);
    memcpy(&debug_data[10], (void*)&eso_z3,       4);
    memcpy(&debug_data[14], (void*)&eso_z2,       4);
    memcpy(&debug_data[18], (void*)&esf_e1_d,     4);
    memcpy(&debug_data[22], (void*)&eso_z1_d,     4);
    memcpy(&debug_data[26], (void*)&yaw_actual,   4);
    memcpy(&debug_data[30], (void*)&yaw_dps,      4);
    memcpy(&debug_data[34], (void*)&yaw_imu_d,    4);
    memcpy(&debug_data[38], (void*)&yaw_enc_raw_d,4);
    HAL_UART_Transmit_DMA(&huart7, debug_data, 42);
#elif defined(DEBUG_FRAME_SYSID)
    /* ===== 系统辨识调试帧 (30B) =====
     * [0-1]   0xAA 0xBB  帧头
     * [2-5]   float       yaw_angle_raw       YAW实际角度 (rad, DM电机回传raw)
     * [6-9]   float       yaw_torque_raw      YAW电机力矩 (Nm, raw)
     * [10-13] float       yaw_vel_motor       YAW角速度电机回传 (rad/s, raw)
     * [14-17] float       yaw_angle_imu       YAW角度上板IMU (rad)
     * [18-21] float       yaw_vel_imu         YAW角速度上板IMU (rad/s)
     * [22-25] float       target_torque       目标力矩 (Nm, 阶跃注入值)
     * [26-29] float       target_velocity     目标速度 (rad/s)
     */
    extern volatile float gimbal_yaw_rx_d;
    static uint8_t sysid_data[30];

    float yaw_angle_raw  = DMyawMotorRec.pos_d;                          /* rad */
    float yaw_torque_raw = DMyawMotorRec.toq;                            /* Nm */
    float yaw_vel_motor  = DMyawMotorRec.vel_radps;                      /* rad/s */
    float yaw_angle_imu  = gimbal_yaw_rx_d * 0.0174533f;                 /* deg→rad */
    float yaw_vel_imu    = gimbal_yaw_dps_rx * 0.0174533f;               /* deg/s→rad/s */

    /* 降采样: 1kHz→500Hz, 隔帧发送 */
    static uint8_t subsample = 0;
    subsample = !subsample;
    if(subsample)
    {
        sysid_data[0] = 0xAA;
        sysid_data[1] = 0xBB;
        memcpy(&sysid_data[2],  (void*)&yaw_angle_raw,  4);
        memcpy(&sysid_data[6],  (void*)&yaw_torque_raw, 4);
        memcpy(&sysid_data[10], (void*)&yaw_vel_motor,  4);
        memcpy(&sysid_data[14], (void*)&yaw_angle_imu,  4);
        memcpy(&sysid_data[18], (void*)&yaw_vel_imu,    4);
#ifdef TEST_YAW
        {
            extern volatile SysIDTest g_sysid_yaw;
            float tt = g_sysid_yaw.out_torque_nm;
            float tv = g_sysid_yaw.out_velocity_radps;
            memcpy(&sysid_data[22], (void*)&tt, 4);
            memcpy(&sysid_data[26], (void*)&tv, 4);
        }
#else
        {
            float tt = 0.0f, tv = 0.0f;
            memcpy(&sysid_data[22], (void*)&tt, 4);
            memcpy(&sysid_data[26], (void*)&tv, 4);
        }
#endif
        HAL_UART_Transmit_DMA(&huart7, sysid_data, 30);
    }
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

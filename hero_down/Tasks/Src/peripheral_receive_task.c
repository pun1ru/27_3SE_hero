#include "tim.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "general_task_include.h"
#include "dt7_remote_driver.h"
#include "VT13_rc_ctrl.h"
#include "ekf_imu_solver.h"
#include "LK_driver.h"
#include "dm_imu.h"
#include "distance_measure.h"
#include "LK_485_driver.h"

/* ==================================================================
 * 共享 UART 接收缓冲区
 * ================================================================== */
uint8_t uart1RecBuffer[32];       /* 上位机(USB CDC) 当前未启用 */
uint8_t uart5RecBuffer[64];       /* 遥控器: DT7=18B / VT3=21B */
uint8_t uart6RecBuffer[160];      /* 裁判系统 最大帧~128B */
uint8_t uart10RecBuffer[49];      /* 激光测距 */
uint8_t uartServentRecBuffer[40]; /* RS485云台→底盘 40B协议帧 */
uint8_t uartMasterRecBuffer[64];  /* RS485底盘→云台 */

/* ==================================================================
 * RemoteRecTask — 遥操作指令接收
 * ================================================================== */

/* ---- 遥控器数据 ---- */
EventGroupHandle_t remoteRecEventGroup;

DT7RecData  dt7RecData;
DT7CmdData  dt7CmdData;
const DT7CmdData* _dt7CmdData = &dt7CmdData;
uint8_t     dt7RecBuffer[DT7_RC_FRAME_LEN];
uint8_t     VT3RecBuffer[VT3_RC_FRAME_LEN];
VT13_RC_ctrl_t vt13_rc_ctrl_t;

NormRemoteCmd normRemoteCmd;
const NormRemoteCmd* _normRemoteCmd = &normRemoteCmd;

static void NormRemoteCmdInit(NormRemoteCmd* norm_remote_cmd);
static void DT7ToNormCmd(NormRemoteCmd* norm_remote_cmd, const DT7CmdData* dt7_cmd_data);

const TickType_t RCDelay = pdMS_TO_TICKS(500);

void RemoteRecTask(void* argument)
{
    static EventBits_t currentEventGroupBits;
    remoteRecEventGroup = xEventGroupCreate();

    static uint32_t last_tick_count, current_tick_count, this_tick_count;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._remote_rec_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._remote_rec_task = &this_tick_count;

    current_tick_count = last_tick_count = xTaskGetTickCount();
    NormRemoteCmdInit(&normRemoteCmd);

    while (1)
    {
        currentEventGroupBits = xEventGroupWaitBits(remoteRecEventGroup,
            (EVENT_GROUP_BIT_ERROR | EVENT_GROUP_BIT_DT7 | EVENT_GROUP_BIT_VT3),
            pdTRUE, pdFALSE, RCDelay);

        task_counter++;

        if (currentEventGroupBits & EVENT_GROUP_BIT_DT7) {
            DT7RawDataUpdate(&dt7RecData, dt7RecBuffer);
            DT7DataProcess(&dt7CmdData, &dt7RecData);
            DT7ToNormCmd(&normRemoteCmd, &dt7CmdData);
        }
        else if (currentEventGroupBits & EVENT_GROUP_BIT_VT3) {
            VT13_to_rc(VT3RecBuffer, &vt13_rc_ctrl_t);
        }
        else if (currentEventGroupBits & EVENT_GROUP_BIT_ERROR) {
            NormRemoteCmdInit(&normRemoteCmd);
            RemoteRecRestart();
        }
        else {
            NormRemoteCmdInit(&normRemoteCmd);
            RemoteRecRestart();
        }

        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
    }
}

static void NormRemoteCmdInit(NormRemoteCmd* norm_remote_cmd)
{
    memset(norm_remote_cmd, 0, sizeof(NormRemoteCmd));
    norm_remote_cmd->remote_source = ERROR_RECEIVE;
}

void RemoteRecRestart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5RecBuffer, sizeof(uart5RecBuffer));
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

static void DT7ToNormCmd(NormRemoteCmd* norm_remote_cmd, const DT7CmdData* dt7_cmd_data)
{
    norm_remote_cmd->remote_source = DT7;
    norm_remote_cmd->RelativeCH.ch0 = dt7_cmd_data->ch0 / DT7_RC_CH_MAX_RELATIVE;
    norm_remote_cmd->RelativeCH.ch1 = dt7_cmd_data->ch1 / DT7_RC_CH_MAX_RELATIVE;
    norm_remote_cmd->RelativeCH.ch2 = dt7_cmd_data->ch2 / DT7_RC_CH_MAX_RELATIVE;
    norm_remote_cmd->RelativeCH.ch3 = dt7_cmd_data->ch3 / DT7_RC_CH_MAX_RELATIVE;
    norm_remote_cmd->RelativeCH.ch4 = dt7_cmd_data->ch4 / DT7_RC_CH_MAX_RELATIVE;
    norm_remote_cmd->Switch.switch_L1 = dt7_cmd_data->switch_left;
    norm_remote_cmd->Switch.switch_R1 = dt7_cmd_data->switch_right;
    memcpy(&norm_remote_cmd->PCKeyBoard, &dt7_cmd_data->PCKeyBoard, sizeof(dt7_cmd_data->PCKeyBoard));
    norm_remote_cmd->PCMouse.mouse_speed_x = dt7_cmd_data->PCMouse.x;
    norm_remote_cmd->PCMouse.mouse_speed_y = dt7_cmd_data->PCMouse.y;
    norm_remote_cmd->PCMouse.mouse_speed_z = dt7_cmd_data->PCMouse.z;
    norm_remote_cmd->PCMouse.mouse_left  = dt7_cmd_data->PCMouse.press_left;
    norm_remote_cmd->PCMouse.mouse_right = dt7_cmd_data->PCMouse.press_right;
}

/* ==================================================================
 * 电机 CAN 接收 —— 数据结构 + FDCAN 回调
 * ================================================================== */

/* ---- fdcan1: yaw + joint×4 + stir ---- */
DMJ4310MotorRec DMyawMotorRec;
const DMJ4310MotorRec* _DMyawMotorRec = &DMyawMotorRec;

DMJ4310MotorRec jointMotorEec[4];
const DMJ4310MotorRec* _jointMotorEec = jointMotorEec;

DMJ4310MotorRec stirMotorRec;
const DMJ4310MotorRec* _stirMotorRec = &stirMotorRec;

/* ---- fdcan2: chassis×4 + caterpillar×2 + superCap ---- */
DJIGMotorRec chassisMotorRec[CHASSIS_MOTOR_NUM];
const DJIGMotorRec* _chassisMotorRec = chassisMotorRec;

DJIGMotorRec yawMotorRec;
const DJIGMotorRec* _yawMotorRec = &yawMotorRec;

DJIGMotorRec fricMotorRec[FRIC_MOTOR_NUM];
const DJIGMotorRec* _fricMotorRec = fricMotorRec;

DJIGMotorRec pitchMotorRec;
const DJIGMotorRec* _pitchMotorRec = &pitchMotorRec;

DJIGMotorRec smallpitchMotorRec;
const DJIGMotorRec* _smallpitchMotorRec = &smallpitchMotorRec;

DMJ4310MotorRec caterpillarMotorRec[2];
const DMJ4310MotorRec* _caterpillarMotorRec = caterpillarMotorRec;

SuperCapacity superCapacity;
const SuperCapacity* _superCapacity = &superCapacity;

long can_heavy = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can_heavy++;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t aData[8];
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, aData);
            switch (RxHeader.Identifier)
            {
            case 0x017:
                DMyawMotorRec.frame_counter++;
                DMyawMotorRec.id    = (aData[0]) & 0x0F;
                DMyawMotorRec.state = (aData[0]) >> 4;
                DMyawMotorRec.p_int = (aData[1] << 8) | aData[2];
                DMyawMotorRec.v_int = (aData[3] << 4) | (aData[4] >> 4);
                DMyawMotorRec.t_int = ((aData[4] & 0xF) << 8) | aData[5];
                DMyawMotorRec.pos_d    = -uint_to_float(DMyawMotorRec.p_int, -(DM_YAW_MAX_ENCODE_D), +(DM_YAW_MAX_ENCODE_D), 16);
                DMyawMotorRec.vel_radps = uint_to_float(DMyawMotorRec.v_int, -45.0, 45.0, 12);
                DMyawMotorRec.toq      = uint_to_float(DMyawMotorRec.t_int, -12.0, 12.0, 12);
                DMyawMotorRec.Tmos     = (float)(aData[6]);
                DMyawMotorRec.Tcoil    = (float)(aData[7]);
                break;

            case 0x011: case 0x012: case 0x013: case 0x014:
            {
                uint8_t idx = RxHeader.Identifier - 0x011;
                jointMotorEec[idx].frame_counter++;
                jointMotorEec[idx].id    = (aData[0]) & 0x0F;
                jointMotorEec[idx].state = (aData[0]) >> 4;
                jointMotorEec[idx].p_int = (aData[1] << 8) | aData[2];
                jointMotorEec[idx].v_int = (aData[3] << 4) | (aData[4] >> 4);
                jointMotorEec[idx].t_int = ((aData[4] & 0xF) << 8) | aData[5];
                jointMotorEec[idx].pos_d    = uint_to_float(jointMotorEec[idx].p_int, -(DM_YAW_MAX_ENCODE_D), +(DM_YAW_MAX_ENCODE_D), 16);
                jointMotorEec[idx].vel_radps = uint_to_float(jointMotorEec[idx].v_int, -10.0, 10.0, 12);
                jointMotorEec[idx].toq      = uint_to_float(jointMotorEec[idx].t_int, -28.0, 28.0, 12);
                jointMotorEec[idx].Tmos     = (float)(aData[6]);
                jointMotorEec[idx].Tcoil    = (float)(aData[7]);
                break;
            }

            case 0x018:
                stirMotorRec.frame_counter++;
                stirMotorRec.id    = (aData[0]) & 0x0F;
                stirMotorRec.state = (aData[0]) >> 4;
                stirMotorRec.p_int = (aData[1] << 8) | aData[2];
                stirMotorRec.v_int = (aData[3] << 4) | (aData[4] >> 4);
                stirMotorRec.t_int = ((aData[4] & 0xF) << 8) | aData[5];
                stirMotorRec.pos_d    = uint_to_float(stirMotorRec.p_int, -(DM_MOTO_MAX_ENCODE_D), +(DM_MOTO_MAX_ENCODE_D), 16);
                stirMotorRec.vel_radps = uint_to_float(stirMotorRec.v_int, -30.0, 30.0, 12);
                stirMotorRec.toq      = uint_to_float(stirMotorRec.t_int, -10.0, 10.0, 12);
                stirMotorRec.Tmos     = (float)(aData[6]);
                stirMotorRec.Tcoil    = (float)(aData[7]);
                break;
            }
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t aData[8];
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0U)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, aData);
            if (hfdcan == &hfdcan2) {
                switch (RxHeader.Identifier) {
                case 0x201: case 0x202: case 0x203: case 0x204:
                {
                    uint8_t idx = RxHeader.Identifier - 0x201;
                    chassisMotorRec[idx].frame_counter++;
                    chassisMotorRec[idx].mechanical_angle      = aData[0] << 8 | aData[1];
                    chassisMotorRec[idx].mechanical_speed_rpm  = aData[2] << 8 | aData[3];
                    chassisMotorRec[idx].torque_current_real   = aData[4] << 8 | aData[5];
                    chassisMotorRec[idx].motor_temperature_d   = aData[6];
                    break;
                }
                case 0x015: case 0x016:
                {
                    uint8_t idx = RxHeader.Identifier - 0x015;
                    caterpillarMotorRec[idx].frame_counter++;
                    caterpillarMotorRec[idx].id    = (aData[0]) & 0x0F;
                    caterpillarMotorRec[idx].state = (aData[0]) >> 4;
                    caterpillarMotorRec[idx].p_int = (aData[1] << 8) | aData[2];
                    caterpillarMotorRec[idx].v_int = (aData[3] << 4) | (aData[4] >> 4);
                    caterpillarMotorRec[idx].t_int = ((aData[4] & 0xF) << 8) | aData[5];
                    caterpillarMotorRec[idx].pos_d    = uint_to_float(caterpillarMotorRec[idx].p_int, -(DM_MOTO_MAX_ENCODE_D), +(DM_MOTO_MAX_ENCODE_D), 16);
                    caterpillarMotorRec[idx].vel_radps = uint_to_float(caterpillarMotorRec[idx].v_int, -45.0, 45.0, 12);
                    caterpillarMotorRec[idx].toq      = uint_to_float(caterpillarMotorRec[idx].t_int, -18.0, 18.0, 12);
                    caterpillarMotorRec[idx].Tmos     = (float)(aData[6]);
                    caterpillarMotorRec[idx].Tcoil    = (float)(aData[7]);
                    break;
                }
                case 0x211:
                    superCapacity.frame_counter++;
                    superCapacity.cap_volt          = (float)(aData[0] | (aData[1] << 8)) / 100.0f;
                    superCapacity.power_limit       = (float)(aData[2] | (aData[3] << 8));
                    superCapacity.real_power        = (float)(aData[4] | (aData[5] << 8));
                    superCapacity.compensated_power = (float)(aData[6] | (aData[7] << 8));
                    break;
                default: break;
                }
            }
            else if (hfdcan == &hfdcan3) {
                switch (RxHeader.Identifier) {
                case 0x211:
                    superCapacity.frame_counter++;
                    superCapacity.cap_volt          = (float)(aData[0] | (aData[1] << 8)) / 100.0f;
                    superCapacity.power_limit       = (float)(aData[2] | (aData[3] << 8));
                    superCapacity.real_power        = (float)(aData[4] | (aData[5] << 8));
                    superCapacity.compensated_power = (float)(aData[6] | (aData[7] << 8));
                    break;
                default: break;
                }
            }
        }
    }
}

/* ==================================================================
 * RS485 云台接收数据（gimbal yaw/pitch/fric 回传）
 * ================================================================== */
volatile float    gimbal_yaw_rx_d          = 0.0f;
volatile float    gimbal_yaw_dps_rx        = 0.0f;
volatile float    gimbal_pitch_dps_rx       = 0.0f;
volatile float    gimbal_yaw_target_rx_d   = 0.0f;
volatile float    gimbal_pitch_target_rx_d  = 0.0f;
volatile float    gimbal_pitch_rx_d         = 0.0f;
volatile int16_t  gimbal_fric_rpm_rx       = 0;
volatile int16_t  gimbal_fric_rpm_rx_arr[6] = {0};
volatile uint8_t  gimbal_yaw_rx_valid      = 0;

/* ==================================================================
 * IMUTask — 板载 IMU 姿态解算
 * ================================================================== */

/* ---- IMU 数据与解算 ---- */
IMUUseEKFSolver imuUseEKFSolver;
const IMUUseEKFSolver* _onboardIMUUseEKF = &imuUseEKFSolver;
IMURecData imuRecData;
const IMURecData* _onboardIMURecForEKF = &imuRecData;

Pose gimbalPose;
const Pose* _gimbalPose = &gimbalPose;

Pose externalRecPose;

/* ---- IMU 温控 PID ---- */
#define OUT_STANDING_TEMPERATURE 40.0f
static float temp_kp = 100.f, temp_ki = 100.f, temp_kd = 10.0f;
#define MAX_OUT 500
static float temp_out = 0, temp_err = 0, temp_err_l = 0, temp_err_ll = 0;

static void OnboardIMUTemperatureControl(float real_temperature)
{
    temp_err_ll = temp_err_l;
    temp_err_l  = temp_err;
    temp_err    = OUT_STANDING_TEMPERATURE - imuRecData.temperature;
    temp_out    = temp_kp * temp_err + temp_ki * (temp_err + temp_err_l + temp_err_ll) + temp_kd * (temp_err - temp_err_l);
    if (temp_out > MAX_OUT) temp_out = MAX_OUT;
    if (temp_out < 0)      temp_out = 0.f;
    htim3.Instance->CCR4 = (uint16_t)temp_out;
}

/* ---- IMU 陀螺仪零偏校准 ---- */
#define GYRO_BIAS_CALIB_TARGET_TEMP   39.0f
#define GYRO_BIAS_CALIB_TOTAL_SAMPLE  5000
#define GYRO_BIAS_CALIB_DECIMATE      10

static float    g_gyro_bias_sum[3]    = {0};
static float    g_gyro_bias_result[3] = {0};
static uint16_t g_gyro_bias_sample_cnt  = 0;
static uint8_t  g_gyro_bias_state       = 0;
static uint8_t  g_gyro_bias_decimate_cnt = 0;

static void GetIMUGyroBiasCalibration(IMURecData* imu_data_rec)
{
    if (g_gyro_bias_state == 2) return;

    if (g_gyro_bias_state == 0) {
        if (imu_data_rec->temperature >= GYRO_BIAS_CALIB_TARGET_TEMP) {
            g_gyro_bias_state = 1;
            g_gyro_bias_sample_cnt = 0;
            g_gyro_bias_decimate_cnt = 0;
            g_gyro_bias_sum[0] = 0.0f;
            g_gyro_bias_sum[1] = 0.0f;
            g_gyro_bias_sum[2] = 0.0f;
        }
        return;
    }

    g_gyro_bias_decimate_cnt++;
    if (g_gyro_bias_decimate_cnt < GYRO_BIAS_CALIB_DECIMATE) return;
    g_gyro_bias_decimate_cnt = 0;

    g_gyro_bias_sum[0] += imu_data_rec->gyro[0] + imu_data_rec->gyro_offset[0];
    g_gyro_bias_sum[1] += imu_data_rec->gyro[1] + imu_data_rec->gyro_offset[1];
    g_gyro_bias_sum[2] += imu_data_rec->gyro[2] + imu_data_rec->gyro_offset[2];
    g_gyro_bias_sample_cnt++;

    if (g_gyro_bias_sample_cnt >= GYRO_BIAS_CALIB_TOTAL_SAMPLE) {
        g_gyro_bias_result[0] = g_gyro_bias_sum[0] / (float)GYRO_BIAS_CALIB_TOTAL_SAMPLE;
        g_gyro_bias_result[1] = g_gyro_bias_sum[1] / (float)GYRO_BIAS_CALIB_TOTAL_SAMPLE;
        g_gyro_bias_result[2] = g_gyro_bias_sum[2] / (float)GYRO_BIAS_CALIB_TOTAL_SAMPLE;
        imu_data_rec->gyro_offset[0] = g_gyro_bias_result[0];
        imu_data_rec->gyro_offset[1] = g_gyro_bias_result[1];
        imu_data_rec->gyro_offset[2] = g_gyro_bias_result[2];
        g_gyro_bias_state = 2;
    }
}

/* ---- 姿态更新 ---- */
#define X 0
#define Y 1
#define Z 2

static void PoseUpdateFromIMU(Pose* pose, const IMUUseEKFSolver* imu_use_ekf)
{
#if defined ONBOARD_EKF_SOLVE
    pose->pitch_radps = -imu_use_ekf->Gyro[X];
    pose->roll_radps  =  imu_use_ekf->Gyro[Y];
    pose->yaw_radps   = -imu_use_ekf->Gyro[Z];

    pose->pitch_d = -imu_use_ekf->Pitch_d;
    pose->yaw_d   = -imu_use_ekf->Yaw_d;
    pose->roll_d  =  imu_use_ekf->Roll_d;

    pose->accel_x = imu_use_ekf->MotionAccel_n[X];
    pose->accel_y = imu_use_ekf->MotionAccel_n[Y];
    pose->accel_z = imu_use_ekf->MotionAccel_n[Z];
#endif
    GimbalPoseUpdate(pose->pitch_d, pose->pitch_radps,
                     pose->yaw_d, pose->yaw_radps,
                     pose->roll_d, pose->roll_radps);
}

/* ---- IMU 任务 ---- */
void IMUTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count = 0;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._imu_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._imu_task    = &this_tick_count;

    current_tick_count = last_tick_count = xTaskGetTickCount();
    while (1)
    {
#if defined ONBOARD_EKF_SOLVE
        IMUSolverUseEKFUserFunc(&imuUseEKFSolver, &imuRecData);
#endif
        OnboardIMUTemperatureControl(imuRecData.temperature);
        PoseUpdateFromIMU(&gimbalPose, &imuUseEKFSolver);

        task_counter++;
        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;

        extern TaskHandle_t estimateTaskHandle;
        if (estimateTaskHandle != NULL)
            xTaskNotifyGive(estimateTaskHandle);

        /* 控制链监控：记录 IMU 通知发出的时刻 */
        g_chain_timer.cyc_imu_notify = DWT->CYCCNT;

        vTaskDelayUntil(&current_tick_count, IMU_TASK_PERIOD_SET);
    }
}

/* ==================================================================
 * UpperPCCommTask — 算法上位机通信
 * ================================================================== */
UpperComputerComm upperComputerComm;
const UpperComputerComm* _upperComputerComm = &upperComputerComm;

extern uint8_t shit_last_PC_Receive_shoot_mode;

void UpperCommRecHandler(uint8_t* rec_buf, uint32_t size)
{
    if (UPPER_PC_COMM_REC_SOF == rec_buf[0]
        && UPPER_PC_COMM_REC_EOF == rec_buf[size - 1]
        && size == UPPER_PC_COMM_REC_LENGTH)
    {
        shit_last_PC_Receive_shoot_mode = upperComputerComm.Receive.shoot_mode;
        memcpy(&upperComputerComm.Receive, rec_buf, sizeof(upperComputerComm.Receive));
        DoubleEdgeLimiter(upperComputerComm.Receive.target_pitch_angle_d, 0, 30);
        DoubleEdgeLimiter(upperComputerComm.Receive.target_yaw_angle_d, -10, 10);
        upperComputerComm.rec_counter++;
    }
}

void UpperPCCommTask(void* argument)
{
    static uint32_t last_tick_count, current_tick_count, this_tick_count = 0;
    static uint16_t task_counter;
    _taskMonitor->TaskFrameCounterPtr._upper_pc_comm_task = &task_counter;
    _taskMonitor->TaskRunPeriodPtr._upper_pc_comm_task    = &this_tick_count;

    upperComputerComm.Send.sof = UPPER_PC_COMM_REC_SOF;
    upperComputerComm.Send.eof = UPPER_PC_COMM_REC_EOF;

    current_tick_count = last_tick_count = xTaskGetTickCount();
    while (1)
    {
        static uint16_t no_rec_counter = 0;
        static uint16_t last_rec_counter = 0;

        task_counter++;

        if (upperComputerComm.rec_counter == last_rec_counter) {
            no_rec_counter++;
            if (no_rec_counter > 500)
                no_rec_counter = 0;
        }
        else {
            no_rec_counter = 0;
            last_rec_counter = upperComputerComm.rec_counter;
        }

        upperComputerComm.Send.self_team =
            (ext_game_robot_status.robot_id < 50) ? RED_TEAM_FRAME : BLUE_TEAM_FRAME;

        upperComputerComm.Send.task_mode =
            (_robotState->sniper == SNIPER_OFF) ? 0 : 2;

        upperComputerComm.Send.bullet_speed  = ext_shoot_data.initial_speed;
        upperComputerComm.Send.gimbal_pitch_d = _gimbalControl->GimbalEstimate.pitch_angle_d;
        upperComputerComm.Send.gimbal_yaw_d   = _gimbalControl->GimbalEstimate.yaw_angle_d;
        upperComputerComm.Send.gimbal_yaw_dps = _gimbalControl->GimbalEstimate.yaw_angular_velocity_dps;

#ifdef UPPER_PC_TRANSMIT_ENABLE
        CDC_Transmit_HS((uint8_t*)&upperComputerComm.Send, UPPER_PC_COMM_SEND_LENGTH);
#endif

        current_tick_count = xTaskGetTickCount();
        this_tick_count = current_tick_count - last_tick_count;
        last_tick_count = current_tick_count;
        vTaskDelayUntil(&current_tick_count, UPPER_COMM_TASK_PERIOD_SET);
    }
}

/* ==================================================================
 * 外设初始化 + UART/CAN 回调
 * ================================================================== */

/* ---- 激光串口接收 ---- */
uint8_t distance_buffer[49];

/* ---- 外设接收初始化 ---- */
void PeripheralRecEnable(void)
{
    DT7RemoteRecEnable(&RC_UART, uart5RecBuffer);
    can_bsp_init();

#if defined ONBOARD_EKF_SOLVE
    IMUSolverUseEKFInitialize(&imuUseEKFSolver, &imuRecData, IMU_TASK_PERIOD_SET / 1000.0f);
#endif

    HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, sizeof(uart6RecBuffer));
    __HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&LASER_UART, uart10RecBuffer, LASER_UART_LENGTH);
    __HAL_DMA_DISABLE_IT(LASER_UART.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&SERVENT_485_UART, uartServentRecBuffer, sizeof(uartServentRecBuffer));
    __HAL_DMA_DISABLE_IT(SERVENT_485_UART.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&MASTER_485_UART, uartMasterRecBuffer, sizeof(uartMasterRecBuffer));
    __HAL_DMA_DISABLE_IT(MASTER_485_UART.hdmarx, DMA_IT_HT);
}

/* ---- UART DMA 不定长接收回调 ---- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* 激光测距 */
    if (huart == &LASER_UART) {
        memcpy(distance_buffer, uart10RecBuffer, Size);
        distance_datacheck(distance_buffer);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer, sizeof(uart10RecBuffer));
        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    }

    /* 遥控器 */
    if (huart == &RC_UART) {
        if (DT7_RC_FRAME_LEN == Size) {
            memcpy(dt7RecBuffer, uart5RecBuffer, Size);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xEventGroupSetBitsFromISR(remoteRecEventGroup, EVENT_GROUP_BIT_DT7, &xHigherPriorityTaskWoken);
        }
        if (Size == VT3_RC_FRAME_LEN) {
            memcpy(VT3RecBuffer, uart5RecBuffer, Size);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xEventGroupSetBitsFromISR(remoteRecEventGroup, EVENT_GROUP_BIT_VT3, &xHigherPriorityTaskWoken);
        }
        memset(uart5RecBuffer, 0, sizeof(uart5RecBuffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&RC_UART, uart5RecBuffer, sizeof(uart5RecBuffer));
        __HAL_DMA_DISABLE_IT(RC_UART.hdmarx, DMA_IT_HT);
    }

    /* 裁判系统 */
    if (huart == &REFEREE_UART) {
        uint8_t temp_buffer[Size];
        memcpy(temp_buffer, uart6RecBuffer, Size);
        memset(uart6RecBuffer, 0, sizeof(uart6RecBuffer));
        RefereeReceive(Size, temp_buffer);
        HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, sizeof(uart6RecBuffer));
        __HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);
    }

    /* RS485 云台→底盘 */
    if (huart == &SERVENT_485_UART) {
        static uint32_t rs485_rx_cnt = 0, rs485_ok_cnt = 0;
        static uint16_t rs485_last_size = 0;
        rs485_rx_cnt++;
        rs485_last_size = Size;

        gimbal_yaw_rx_valid = 0;
        if (Size >= 40U
            && uartServentRecBuffer[0] == 0xA5
            && uartServentRecBuffer[1] == 0x5A
            && uartServentRecBuffer[38] == 0x0D
            && uartServentRecBuffer[39] == 0x0A)
        {
            memcpy((void*)&gimbal_yaw_rx_d,        &uartServentRecBuffer[2],  sizeof(float));
            memcpy((void*)&gimbal_pitch_rx_d,       &uartServentRecBuffer[6],  sizeof(float));
            memcpy((void*)&gimbal_yaw_dps_rx,       &uartServentRecBuffer[10], sizeof(float));
            memcpy((void*)&gimbal_pitch_dps_rx,     &uartServentRecBuffer[14], sizeof(float));
            memcpy((void*)&gimbal_yaw_target_rx_d,  &uartServentRecBuffer[18], sizeof(float));
            memcpy((void*)&gimbal_pitch_target_rx_d, &uartServentRecBuffer[22], sizeof(float));
            for (uint8_t i = 0; i < 6; i++)
                memcpy((void*)&gimbal_fric_rpm_rx_arr[i], &uartServentRecBuffer[26 + i * 2], sizeof(int16_t));
            gimbal_fric_rpm_rx  = gimbal_fric_rpm_rx_arr[0];
            gimbal_yaw_rx_valid = 1;
            rs485_ok_cnt++;
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&SERVENT_485_UART, uartServentRecBuffer, sizeof(uartServentRecBuffer));
        __HAL_DMA_DISABLE_IT(SERVENT_485_UART.hdmarx, DMA_IT_HT);
    }
}

/* ---- UART 错误回调 ---- */
extern QueueHandle_t g_musicQueue;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    HAL_UART_AbortReceive(huart);

    if (huart == &huart5)
        NormRemoteCmdInit(&normRemoteCmd);

    if (huart->Instance == USART10) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, uart10RecBuffer, sizeof(uart10RecBuffer));
        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == REFEREE_UART.Instance) {
        HAL_UARTEx_ReceiveToIdle_DMA(&REFEREE_UART, uart6RecBuffer, sizeof(uart6RecBuffer));
        __HAL_DMA_DISABLE_IT(REFEREE_UART.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == SERVENT_485_UART.Instance) {
        HAL_UARTEx_ReceiveToIdle_DMA(&SERVENT_485_UART, uartServentRecBuffer, sizeof(uartServentRecBuffer));
        __HAL_DMA_DISABLE_IT(SERVENT_485_UART.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == MASTER_485_UART.Instance) {
        HAL_UARTEx_ReceiveToIdle_DMA(&MASTER_485_UART, uartMasterRecBuffer, sizeof(uartMasterRecBuffer));
        __HAL_DMA_DISABLE_IT(MASTER_485_UART.hdmarx, DMA_IT_HT);
    }

    short temp = 1;
    BaseType_t xHigherPriorityTaskWoken = 0;
    xQueueSendFromISR(g_musicQueue, &temp, &xHigherPriorityTaskWoken);
}

/* ---- FDCAN 错误回调 ---- */
#define FDCAN_ERROR_MASK (FDCAN_IR_ELO | FDCAN_IR_WDI | FDCAN_IR_PEA | FDCAN_IR_PED | FDCAN_IR_ARA)

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    extern CANTxMonitor canMonitor[3];
    uint8_t idx = 0;
    if (hfdcan == &hfdcan2) idx = 1;
    else if (hfdcan == &hfdcan3) idx = 2;
    canMonitor[idx].err_callback++;

    __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_ERROR_MASK);

    if (canMonitor[idx].err_callback % 1000 == 0) {
        canMonitor[idx].err_reinit++;
        HAL_FDCAN_DeInit(hfdcan);
        HAL_FDCAN_Init(hfdcan);
        can_filter_init();
        HAL_FDCAN_Start(hfdcan);

        if (hfdcan == &hfdcan1)
            HAL_FDCAN_ActivateNotification(hfdcan, 0x00038001, NULL);
        else if (hfdcan == &hfdcan2)
            HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_TX_EVT_FIFO_NEW_DATA, NULL);
        else
            HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, NULL);
    }

    short temp = 2;
    BaseType_t xHigherPriorityTaskWoken = 0;
    xQueueSendFromISR(g_musicQueue, &temp, &xHigherPriorityTaskWoken);
}
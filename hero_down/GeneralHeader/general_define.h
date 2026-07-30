#ifndef GENERAL_DEFINE_H_
#define GENERAL_DEFINE_H_

/* Build and debug switches ================================================= */
#define DEBUG_STOP_IWDG() __HAL_DBGMCU_FREEZE_IWDG()
#define DEBUG_RESUME_IWDG() __HAL_DBGMCU_UNFREEZE_IWDG()

#ifndef YAW_ANGLE_FROM_MOTO
#define YAW_ANGLE_FROM_IMU 1
#endif

#define UPPER_PC_TRANSMIT_ENABLE
#define DEBUG_MSG_ENABLE
#define MATCH_MODE
#define MUSIC_OFF

/* Common values =========================================================== */
#define LEFT 0
#define RIGHT 1
#define UP 2
#define LEFT1 3
#define RIGHT1 4
#define UP1 5

/* Normalized remote switch values ======================================== */
#define NORM_RC_SW_UP 1
#define NORM_RC_SW_MID 3
#define NORM_RC_SW_DOWN 2

/* Normalized remote input ================================================= */
#define rc_ch0 (_normRemoteCmd->RelativeCH.ch0)
#define rc_ch1 (_normRemoteCmd->RelativeCH.ch1)
#define rc_ch2 (_normRemoteCmd->RelativeCH.ch2)
#define rc_ch3 (_normRemoteCmd->RelativeCH.ch3)
#define rc_ch4 (_normRemoteCmd->RelativeCH.ch4)
#define rc_source (_normRemoteCmd->remote_source)

#define key_w (_normRemoteCmd->PCKeyBoard.level_key_W)
#define key_a (_normRemoteCmd->PCKeyBoard.level_key_A)
#define key_s (_normRemoteCmd->PCKeyBoard.level_key_S)
#define key_d (_normRemoteCmd->PCKeyBoard.level_key_D)
#define key_e (_normRemoteCmd->PCKeyBoard.level_key_E)
#define key_shift (_normRemoteCmd->PCKeyBoard.level_key_SHIFT)
#define key_ctrl (_normRemoteCmd->PCKeyBoard.level_key_CTRL)

#define rc_mouse_speed_x (_normRemoteCmd->PCMouse.mouse_speed_x)
#define rc_mouse_speed_y (_normRemoteCmd->PCMouse.mouse_speed_y)
#define key_ad_released ((!key_a) && (!key_d))
#define key_wse_released ((!key_w) && (!key_s) && (!key_e))
#define key_wasd_released ((!key_w) && (!key_a) && (!key_s) && (!key_d))
#define key_ad_direction (key_d - key_a)
#define key_ws_direction (key_w - key_s)

/* Input edge detection ===================================================== */
#ifndef TRUE
#define TRUE 1U
#endif

#ifndef FALSE
#define FALSE 0U
#endif

#define UPTRIG(now, last) ((now == TRUE) && (last == FALSE))
#define DOWNTRIG(now, last) ((now == FALSE) && (last == TRUE))

/* Task periods, milliseconds ============================================= */
#define MONITOR_TASK_PERIOD_SET 50
#define STATE_TASK_PERIOD_SET 10
#define DECISION_TASK_PERIOD_SET 10
#define CONTROL_TASK_PERIOD_SET 3
#define IMU_TASK_PERIOD_SET 2
#define DEBUG_TASK_PERIOD_SET 1
#define UPPER_COMM_TASK_PERIOD_SET 4
#define UI_OPERATION_TASK_PERIOD_SET 3
#define MUSIC_TASK_PERIOD_SET 20
#define ESTIMATE_TASK_PERIOD_SET 1

#endif

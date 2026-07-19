#ifndef _GENERAL_CONFIG_LABEL_H_
#define _GENERAL_CONFIG_LABEL_H_

#define DEBUG_STOP_IWDG() __HAL_DBGMCU_FREEZE_IWDG()
#define DEBUG_RESUME_IWDG() __HAL_DBGMCU_UNFREEZE_IWDG()

#ifndef YAW_ANGLE_FROM_MOTO 
#define YAW_ANGLE_FROM_IMU 1
#endif
/*串口调试和算法通信占用同一个uart，不能同时使用*/
/*有两种方案可以同时使用：一个是用拓展版，或者关闭激光*/
#define UPPER_PC_TRANSMIT_ENABLE
#define DEBUG_MSG_ENABLE
//#define DEBUG_FRAME_CTRL      /* ADRC调试帧 — yaw ADRC已搬迁至hero_up，此帧暂不可用 */

/*比赛模式*/
#define MATCH_MODE

#define MUSIC_OFF

//激光开关宏
#define LASER_ON() 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3)
#define LASER_OFF() HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3)

/*define chassis motor orientation label*/
#define LF 0	//left-front
#define RF 1	//right-front
#define RB 2	//right-back
#define LB 3	//left-back
#define CHASSIS_MOTOR_NUM 4

#define LEFT 0
#define RIGHT 1
#define UP 2                     //hxg

#define LEFT1 3
#define RIGHT1 4
#define UP1 5

#define FRIC_MOTOR_NUM 6

#define ENABLE 1
#define DISABLE 0

/*机器人参数相关*/
#define PI 3.1415926f
#define WHEEL_RADIUS 0.076f
#define ROBOT_CENTER_TO_WHEEL_RADIUS 0.6221f//0.6221f
#define M3508_REDUCTION_RATIO (268/17)
#define WHEEL_RPM_TO_WHEEL_MPS (2 * PI * WHEEL_RADIUS / 60.0f / M3508_REDUCTION_RATIO) // * MOTOR_MECHENICAL_SPEED
#define WHEEL_MPS_TO_WHEEL_RPM (1 / (WHEEL_RPM_TO_WHEEL_MPS*1.0f)) // * m/s speed
#define ROBOT_RPS_TO_WHEEL_MPS (2 * PI * ROBOT_CENTER_TO_WHEEL_RADIUS) // * RPS 
#define WHEEL_MPS_TO_ROBOT_RPS (1 / (ROBOT_RPS_TO_WHEEL_MPS*1.0f))
#define GM6020_FULL_CIRCLE_MECHENICAL_ANGLE 8191.0f
#define LK_FULL_CIRCLE_MECHENICAL_ANGLE 65535.0f
#define DM_YAW_MAX_ENCODE_D 3.14
#define DM_MOTO_MAX_ENCODE_D 6*60*17//拨盘用

/*通用遥控器拨杆键值，所有遥控器需统一*/
#define NORM_RC_SW_UP 1
#define NORM_RC_SW_MID 3
#define NORM_RC_SW_DOWN 2

/*电机相关*/
#define M3508_MAX_OUTPUT_CURRENT 16000
#define TEMP_SHOOT_3508_CURRENT_MAX 8000
#define GM6020_MAX_OUTPUT_VOLTAGE 30000
#define GM6020_MAX_OUTPUT_CURRENT 16000
#define DM_MAX_OUTPUT_CURRENT 10
#define LK_MAX_OUTPUT_CURRENT 2000
/*当前所开任务线程数-除软件看门狗任务外*/
#define CREATE_TASK_NUM 9+1

/*任务编号*/
#define REMOTE_RECEIVE_TASK_NUM 0
#define STATE_TASK_NUM 1
#define DECISION_TASK_NUM 2
#define CONTROL_TASK_NUM 3
#define IMU_TASK_NUM 4
#define DEBUG_TASK_NUM 5
#define UPPER_COMM_TASK_NUM 6
#define UI_OPERATION_TASK_NUM 7
#define MUSIC_TASK_NUM 8
#define ESTIMATE_TASK_NUM 9
/*设定任务周期-如果有定周期执行的任务*/
//所有任务周期设置不能大于软件看门狗MONITOR_TASK_PERIOD_SET
#define MONITOR_TASK_PERIOD_SET	50
#define	STATE_TASK_PERIOD_SET 10
#define DECISION_TASK_PERIOD_SET 10
#define CONTROL_TASK_PERIOD_SET	3
#define IMU_TASK_PERIOD_SET	2
#define DEBUG_TASK_PERIOD_SET 1
#define UPPER_COMM_TASK_PERIOD_SET 4
#define UI_OPERATION_TASK_PERIOD_SET 3
#define MUSIC_TASK_PERIOD_SET 20
#define ESTIMATE_TASK_PERIOD_SET 1

/* ==================== CAN 总线电机 ID 宏定义 ==================== */
/* --- hfdcan1 MIT 电机 --- */
#define CAN1_JOINT_LF    0x01    /* 关节左前 */
#define CAN1_JOINT_RF    0x02    /* 关节右前 */
#define CAN1_JOINT_RB    0x03    /* 关节右后 */
#define CAN1_JOINT_LB    0x04    /* 关节左后 */
#define CAN1_YAW         0x07    /* 云台 yaw */
#define CAN1_STIR        0x08    /* 拨盘（维护用 raw ID，非 MIT 模式） */

/* --- hfdcan2 MIT 电机 --- */
#define CAN2_CATERPILLAR_L  0x05    /* 履带左 */
#define CAN2_CATERPILLAR_R  0x06    /* 履带右 */

/* --- 维护电机数组长度 --- */
#define CAN1_MAINTAIN_COUNT  6
#define CAN2_MAINTAIN_COUNT  2

#endif



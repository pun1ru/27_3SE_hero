#ifndef DEVICE_DEFINE_H_
#define DEVICE_DEFINE_H_

/* Peripheral mappings ==================================================== */
#define BUZZER_TIM htim12
#define BUZZER_TIM_CHANNEL TIM_CHANNEL_2
#define BOARD_LED_SPI hspi6

#define REFEREE_UART huart1
#define RC_UART huart5
#define DEBUG_UART huart9
#define LASER_UART huart10
#define PITCH_UART huart8
#define MASTER_485_UART huart2
#define SERVENT_485_UART huart3

#define LASER_ON() HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3)
#define LASER_OFF() HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3)

/* Motor topology and limits ============================================== */
#define LF 0
#define RF 1
#define RB 2
#define LB 3
#define CHASSIS_MOTOR_NUM 4
#define FRIC_MOTOR_NUM 6

#define M3508_REDUCTION_RATIO (268 / 17)
#define GM6020_FULL_CIRCLE_MECHENICAL_ANGLE 8191.0f
#define LK_FULL_CIRCLE_MECHENICAL_ANGLE 65535.0f
#define DM_YAW_MAX_ENCODE_D 3.14f
#define DM_MOTO_MAX_ENCODE_D (6.0f * 60.0f * 17.0f)

#define M3508_MAX_OUTPUT_CURRENT 16000
#define TEMP_SHOOT_3508_CURRENT_MAX 8000
#define GM6020_MAX_OUTPUT_VOLTAGE 30000
#define GM6020_MAX_OUTPUT_CURRENT 16000
#define DM_MAX_OUTPUT_CURRENT 10
#define LK_MAX_OUTPUT_CURRENT 2000

#define GMJ4310MOTOR_ID (0x08U + 0x100U)
#define GMJ4310MASTER_ID 0x018U
#define P_MIN (-3.14f)
#define P_MAX 3.14f
#define V_MIN (-30.0f)
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN (-11.0f)
#define T_MAX 11.0f

/* Motor CAN IDs ========================================================== */
#define CAN1_JOINT_LF 0x01U
#define CAN1_JOINT_RF 0x02U
#define CAN1_JOINT_RB 0x03U
#define CAN1_JOINT_LB 0x04U
#define CAN1_YAW 0x07U
#define CAN1_STIR 0x08U

#define CAN1_JOINT_LF_RX 0x011U
#define CAN1_JOINT_RF_RX 0x012U
#define CAN1_JOINT_RB_RX 0x013U
#define CAN1_JOINT_LB_RX 0x014U
#define CAN1_JOINT_RX_FIRST CAN1_JOINT_LF_RX
#define CAN1_JOINT_RX_LAST CAN1_JOINT_LB_RX
#define CAN1_STIR_RX 0x018U

#define CAN2_CATERPILLAR_L 0x05U
#define CAN2_CATERPILLAR_R 0x06U
#define CAN2_CATERPILLAR_RX_FIRST 0x015U
#define CAN2_CATERPILLAR_RX_LAST 0x016U
#define CAN2_CHASSIS_TX 0x200U
#define CAN2_CHASSIS_LF_RX 0x201U
#define CAN2_CHASSIS_RF_RX 0x202U
#define CAN2_CHASSIS_RB_RX 0x203U
#define CAN2_CHASSIS_LB_RX 0x204U
#define CAN2_CHASSIS_RX_FIRST CAN2_CHASSIS_LF_RX
#define CAN2_CHASSIS_RX_LAST CAN2_CHASSIS_LB_RX
#define SUPERCAP_RX 0x211U
#define SUPERCAP_TX 0x2FFU

#define CAN1_MAINTAIN_COUNT 6
#define CAN2_MAINTAIN_COUNT 2

/* Board-to-board CAN ===================================================== */
#define B2B_CAN hfdcan3
#define B2B_DOWN_BODY_STATE 0x220U
#define B2B_DOWN_GIMBAL_INPUT 0x221U
#define B2B_DOWN_KEYS_SWITCH 0x222U
#define B2B_DOWN_STIR 0x223U
#define B2B_DOWN_SHOOT_STATE 0x224U
#define B2B_UP_GIMBAL_POSE 0x228U
#define B2B_UP_GIMBAL_TARGET 0x229U
#define B2B_UP_FRIC_RPM_A 0x22AU
#define B2B_UP_FRIC_RPM_B 0x22BU
#define B2B_UP_GIMBAL_VELOCITY 0x22CU

#endif

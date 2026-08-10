#ifndef TASK_TRANSMIT_H_
#define TASK_TRANSMIT_H_

#include <stdint.h>

#define UPPER_PC_COMM_SEND_LENGTH (24U + 4U)
#define UPPER_PC_COMM_REC_LENGTH 20U
#define UPPER_PC_COMM_REC_SOF 0x33U
#define UPPER_PC_COMM_REC_EOF 0xEEU

#define NO_AIM_NOR_SHOOT 0x00U
#define AIM_BUT_NOT_SHOOT 0x01U
#define AIM_AND_SHOOT 0x02U

#define AUTO_AIM 0x01U
#define BUFF_AIM 0x02U
#define HANG_AIM 0x03U

#define RED_TEAM_FRAME 0xAAU
#define BLUE_TEAM_FRAME 0xBBU
#define MCU_FRAME_LEN (1U + 21U)

typedef struct {
    uint16_t rec_counter;
    struct {
        uint8_t sof;
        uint8_t reboot;
        uint8_t reserved1[2];
        float target_pitch_angle_d;
        float target_yaw_angle_d;
        float yaw_speed_error;
        uint8_t shoot_mode;
        uint8_t reserved2;
        uint8_t aiming_state;
        uint8_t eof;
    } Receive;
    struct {
        uint8_t sof;
        uint8_t task_mode;
        uint8_t self_team;
        uint8_t cam_target;
        float bullet_speed;
        float gimbal_roll_d;
        float gimbal_yaw_d;
        float gimbal_pitch_d;
        float gimbal_yaw_dps;
        uint8_t reserved[3];
        uint8_t eof;
    } Send;
} UpperComputerComm;

extern const UpperComputerComm *const _upperComputerComm;

void MotorControlCANSend(void);
void DebugTask(void *argument);

#endif

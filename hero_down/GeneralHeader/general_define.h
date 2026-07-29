#ifndef __GENERAL_DEFINE_H_
#define __GENERAL_DEFINE_H_
// #define DEBUG_PCB_EN 1

/* 设备端口映射 ================================================================================*/
// 蜂鸣器
// PB15
#define BUZZER_TIM htim12                // Tim
#define BUZZER_TIM_CHANNEL TIM_CHANNEL_2 // Tim_Channel
// 板载LED SPI6
// WS2812
// PA07
/*各种UART的那个那个*/
#define BOARD_LED_SPI hspi6 // SPI
// 裁判uart，自瞄uart，遥控器uart
#define REFEREE_UART huart1 // 注意这个uart1，手册上写错，最后一页写成uart3了了
#define RC_UART huart5      // 记得改回来
/*8和9都在拓展*/
// #define MINIPC_UART huart7//100 000  //原来7是
#define DEBUG_UART huart9       // 100 000 不行太高了256000
#define LASER_UART huart10      // 9600
#define PITCH_UART huart8       // 这个世界就是一个巨大的唐氏临空 115200
#define MASTER_485_UART huart2  // 115200
#define SERVENT_485_UART huart3 // 9600,改成电磁铁控制

/* 归一化遥控输入 ==============================================================================*/
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

#endif
/*CAN口见transmit*/

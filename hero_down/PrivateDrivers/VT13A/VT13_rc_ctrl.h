#ifndef VT13_RC_CTRL_H
#define VT13_RC_CTRL_H
#include <stdint.h>   // 定义了 uint8_t, uint16_t, int16_t 等精确宽度整数类型
#include "cmsis_compiler.h"
#include <stdbool.h>  // 定义了 bool, true, false
#include <stddef.h>   // 定义了 NULL  
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define VT13_HEADER_SIZE        sizeof(VT13_RC_ctrl_t)
	

/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED_VT13         (  VT13_rc_ctrl.key.v  )
#define    IF_KEY_PRESSED_W_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)     != 0 )
#define    IF_KEY_PRESSED_S_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)     != 0 )
#define    IF_KEY_PRESSED_A_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)     != 0 )
#define    IF_KEY_PRESSED_D_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)     != 0 )
#define    IF_KEY_PRESSED_Q_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)     != 0 )
#define    IF_KEY_PRESSED_E_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)     != 0 )
#define    IF_KEY_PRESSED_G_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)     != 0 )
#define    IF_KEY_PRESSED_X_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)     != 0 )
#define    IF_KEY_PRESSED_Z_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)     != 0 )
#define    IF_KEY_PRESSED_C_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)     != 0 )
#define    IF_KEY_PRESSED_B_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)     != 0 )
#define    IF_KEY_PRESSED_V_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)     != 0 )
#define    IF_KEY_PRESSED_F_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)     != 0 )
#define    IF_KEY_PRESSED_R_VT13       ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)     != 0 )
#define    IF_KEY_PRESSED_CTRL_VT13    ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)  != 0 )
#define    IF_KEY_PRESSED_SHIFT_VT13   ( (VT13_rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT_VT13    (VT13_rc_ctrl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH_VT13    (VT13_rc_ctrl.mouse.press_r == 1)
#define    IF_MOUSE_PRESSED_MID_VT13     (VT13_rc_ctrl.mouse.middle == 1)


/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED_VT13    (VT13_rc_ctrl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED_VT13    (VT13_rc_ctrl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED_VT13    (VT13_rc_ctrl.mouse.z)

typedef struct __attribute__((packed))
{
        struct
        {
                int16_t ch[4];
                uint8_t mode_sw;
                uint8_t stop;
                uint8_t left_button;//fn;
                uint8_t	right_button;
                int16_t wheel;
                uint8_t shutter;
        } rc;
        struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
                uint8_t middle;
        } mouse;
        struct
        {
                uint16_t v;
        } key;
        uint16_t crc16;

} VT13_RC_ctrl_t;


/*遥操作接收数据来源事件组BIT宏定义*/
#define EVENT_GROUP_BIT_ERROR 			(1UL << 0UL)	//传输错误,一段时间内都未接收到正确信号
#define EVENT_GROUP_BIT_DT7 			(1UL << 1UL)	//DT7
#define EVENT_GROUP_BIT_VT3				(1UL<<2UL) //遥控器或者别的什么遥控器比如VT3
/**
 * @brief 当前遥操作信号来源编号
 */
typedef enum
{ERROR_RECEIVE=0, DT7,VT13}RemoteSourceEnum;

/**
 * @brief 由于遥操作数据可能来源于不同遥控器或自定义控制器，为了robotcontrol中拿到的信息能统一，
 *		  将不同遥控器的键位信息归化到统一的键位信息结构体中，便于读取
 */
typedef struct
{
	uint8_t remote_source;
	__PACKED_STRUCT
	{
		unsigned switch_L1   : 2;
		unsigned switch_R1   : 2;
		unsigned reserved 	 : 4;
	}Switch;

	__PACKED_STRUCT
	{
		unsigned level_key_W : 1;
		unsigned level_key_A : 1;
		unsigned level_key_S : 1;
		unsigned level_key_D : 1;
		unsigned level_key_SHIFT : 1;
		unsigned level_key_CTRL  : 1;
		unsigned level_key_Q : 1;
		unsigned level_key_E : 1;

		unsigned level_key_R : 1;
		unsigned level_key_F : 1;
		unsigned level_key_G : 1;
		unsigned level_key_Z : 1;
		unsigned level_key_X : 1;
		unsigned level_key_C : 1;
		unsigned level_key_V : 1;
		unsigned level_key_B : 1;
	}PCKeyBoard;
	struct
	{
		float ch0, ch1, ch2, ch3, ch4;
	}RelativeCH;
	struct
	{
		uint8_t mouse_left;
		uint8_t mouse_right;
		int16_t mouse_speed_x;
		int16_t mouse_speed_y;
		int16_t mouse_speed_z;
	}PCMouse;
}NormRemoteCmd;

extern const NormRemoteCmd* _normRemoteCmd;

extern VT13_RC_ctrl_t VT13_rc_ctrl;
extern const VT13_RC_ctrl_t *get_VT13_remote_control_point(void);


void VT13_to_rc(uint8_t *VT13_buf, VT13_RC_ctrl_t *rc_ctrl);




#endif


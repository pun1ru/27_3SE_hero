/** Include Header Files **/
#include "LK_driver.h"

/*note:LK电机命令除广播模式外均为一对一传输，帧头为0x140 + motor_id*/

/**
 * @description: LK read PID
 * @param {} 
 * @return: void
 * @note:  
 */
void LK_pidRead(uint8_t* adata)
{
	adata[0] = 0x30;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;
}

/**
 * @description: LK write PID to RAM
 * @param {} 
 * @return: void
 * @note:  write PID parameters to RAM, lost when power is off 
 */
void LK_pidWrite_RAM(uint8_t* adata, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
{
	adata[0] = 0x31;		
	adata[1] = 0;
	adata[2] = angle_kp;
	adata[3] = angle_ki;
	adata[4] = speed_kp;
	adata[5] = speed_ki;
	adata[6] = iq_kp;
	adata[7] = iq_ki;
}


/**
 * @description: LK write PID to ROM
 * @param {} 
 * @return: void
 * @note:  write PID parameters to ROM, won't lost when power is off
 */
void LK_pidWrite_ROM(uint8_t* adata, uint8_t angle_kp, uint8_t angle_ki, uint8_t speed_kp, uint8_t speed_ki, uint8_t iq_kp, uint8_t iq_ki)
{
	adata[0] = 0x32;		
	adata[1] = 0;
	adata[2] = angle_kp;
	adata[3] = angle_ki;
	adata[4] = speed_kp;
	adata[5] = speed_ki;
	adata[6] = iq_kp;
	adata[7] = iq_ki;	
}


/**
 * @description: LK read accelaration command
 * @param {} 
 * @return: void
 * @note:  
 */
void LK_accelRead(uint8_t* adata)
{
	adata[0] = 0x33;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;
}


/**
 * @description: LK write accelaration command
 * @param:  {accel} dps/s
 * @return: void
 * @note:  write to RAM, lost when power is off
 */
void LK_accelWrite(uint8_t* adata, int32_t accel)
{
	adata[0] = 0x34;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *(uint8_t*)(&accel);
	adata[5] = *((uint8_t*)(&accel)+1);
	adata[6] = *((uint8_t*)(&accel)+2);
	adata[7] = *((uint8_t*)(&accel)+3);	
}


/**
 * @description: LK read encoder command
 * @param:  
 * @return: void
 * @note:  
 */
void LK_encoderRead(uint8_t* adata)
{
	adata[0] = 0x90;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}


/**
 * @description: LK set encoder zero point command
 * @param:  
 * @return: void
 * @note:  write to ROM
 */
void LK_encoderSet(uint8_t* adata, uint16_t encoder_offset)
{
	adata[0] = 0x91;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = *(uint8_t*)(&encoder_offset);
	adata[7] = *((uint8_t*)(&encoder_offset)+1);
}


/**
 * @description: LK read multi-loop angle command
 * @param:  
 * @return: void
 * @note:  
 */
void LK_MultiLoop_angleRead(uint8_t* adata)
{
	adata[0] = 0x92;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}


/**
 * @description: LK read single-loop angle command
 * @param:  
 * @return: void
 * @note:  
 */
void LK_SingleLoop_angleRead(uint8_t* adata)
{
	adata[0] = 0x94;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}


/**
 * @description: LK read state command
 * @param:  
 * @return: void
 * @note:  read temperature, voltage and error flag
 */
void LK_stateRead_1(uint8_t* adata)
{
	adata[0] = 0x9A;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;		
}


/**
 * @description: LK clear error command
 * @param:  
 * @return: void
 * @note:  
 */
void LK_errorClear(uint8_t* adata)
{
	adata[0] = 0x9B;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}


/**
 * @description: LK read state command
 * @param:  
 * @return: void
 * @note:  read temperature, iq, speed and encoder data
 */
void LK_stateRead_2(uint8_t* adata)
{
	adata[0] = 0x9C;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}

/**
 * @description: LK read state command
 * @param:  
 * @return: void
 * @note:  read temperature and phase current
 */
void LK_stateRead_3(uint8_t* adata)
{
	adata[0] = 0x9D;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}


/**
 * @description: LK shut off the motor, clear current state and command
 * @param:  
 * @return: void
 * @note:  
 */
void LK_Motor_off(uint8_t* adata)
{
	adata[0] = 0x80;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}


/**
 * @description: LK shut off the motor, keep current state and command
 * @param:  
 * @return: void
 * @note:  
 */
void LK_Motor_stop(uint8_t* adata)
{
	adata[0] = 0x81;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;
}


/**
 * @description: LK start running after stopped
 * @param:  
 * @return: void
 * @note:  
 */
void LK_Motor_run(uint8_t* adata)
{
	adata[0] = 0x88;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;	
}


void LK_iqControl(uint8_t* adata, int32_t iq_control)
{
	adata[0] = 0xA1;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *(uint8_t *)(&iq_control);
	adata[5] = *((uint8_t *)(&iq_control)+1);
	adata[6] = 0;
	adata[7] = 0;
}
void LK_powerControl(uint8_t* adata, int32_t power_control)
{	
	adata[0] = 0xA0;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *(uint8_t *)(&power_control);
	adata[5] = *((uint8_t *)(&power_control)+1);
	adata[6] = 0;
	adata[7] = 0;
}
void LK_speedControl(uint8_t* adata, int32_t speed_control)
{
	adata[0] = 0xA2;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *(uint8_t *)(&speed_control);
	adata[5] = *((uint8_t *)(&speed_control)+1);
	adata[6] = *((uint8_t *)(&speed_control)+2);
	adata[7] = *((uint8_t *)(&speed_control)+3);	
}

void LK_MultiLoop_angleControl(uint8_t* adata, int32_t angle_control)
{
	adata[0] = 0xA3;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *(uint8_t *)(&angle_control);
	adata[5] = *((uint8_t *)(&angle_control)+1);
	adata[6] = *((uint8_t *)(&angle_control)+2);
	adata[7] = *((uint8_t *)(&angle_control)+3);	
}

void LK_MultiLoop_angleControl_limited(uint8_t* adata, uint16_t max_speed, int32_t angle_control)
{
	adata[0] = 0xA4;		
	adata[1] = 0;
	adata[2] = *(uint8_t *)(&max_speed);
	adata[3] = *((uint8_t *)(&max_speed)+1);
	adata[4] = *(uint8_t *)(&angle_control);
	adata[5] = *((uint8_t *)(&angle_control)+1);
	adata[6] = *((uint8_t *)(&angle_control)+2);
	adata[7] = *((uint8_t *)(&angle_control)+3);
}


/**
 * @description: LK Single-loop Angle Control command
 * @param {} 
 * @return: void
 * @note: limited by max_speed, Max Acceleration and MaxTorqueCurrent
 */
void LK_SingleLoop_angleControl(uint8_t* adata, uint8_t spin_dir, uint16_t angle_control)
{
	adata[0] = 0xA5;		
	adata[1] = spin_dir;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *(uint8_t *)(&angle_control);
	adata[5] = *((uint8_t *)(&angle_control)+1);
	adata[6] = 0;
	adata[7] = 0;	
}


/**
 * @description: LK Single-loop Angle Control command
 * @param {} 
 * @return: void
 * @note: 
 */
void LK_SingleLoop_angleControl_limited(uint8_t* adata, uint8_t spin_dir, uint16_t max_speed, uint32_t angle_control)
{
	adata[0] = 0xA6;		
	adata[1] = spin_dir;
	adata[2] = *(uint8_t *)(&max_speed);
	adata[3] = *((uint8_t *)(&max_speed)+1);
	adata[4] = *(uint8_t *)(&angle_control);
	adata[5] = *((uint8_t *)(&angle_control)+1);
	adata[6] = *((uint8_t *)(&angle_control)+2);
	adata[7] = *((uint8_t *)(&angle_control)+3);
}


/**
 * @description: LK Angle Increment Control command
 * @param {} 
 * @return: void
 * @note: +/- value for different direction 
 */
void LK_Increment_angleControl(uint8_t* adata, int32_t angle_increment)
{
	adata[0] = 0xA7;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *((uint8_t *)(&angle_increment));
	adata[5] = *((uint8_t *)(&angle_increment)+1);
	adata[6] = *((uint8_t *)(&angle_increment)+2);
	adata[7] = *((uint8_t *)(&angle_increment)+3);
}


/**
 * @description: LK Angle Increment Control command (with max_speed)
 * @param {} 
 * @return: void
 * @note: +/- value for different direction
 * @note: limited by max Acceleration and MaxTorqueCurrent
 */
void LK_Increment_angleControl_limited(uint8_t* adata, uint16_t max_speed, int32_t angle_increment)
{
	adata[0] = 0xA8;		
	adata[1] = 0;
	adata[2] = *((uint8_t *)(&max_speed));
	adata[3] = *((uint8_t *)(&max_speed)+1);
	adata[4] = *((uint8_t *)(&angle_increment));
	adata[5] = *((uint8_t *)(&angle_increment)+1);
	adata[6] = *((uint8_t *)(&angle_increment)+2);
	adata[7] = *((uint8_t *)(&angle_increment)+3);
}

 
/**
 * @brief broadcast mode
 * @note stdid = 0x280
 */

void LK_Multi_iqControl(uint8_t* adata, int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4)
{
	adata[0] = *(uint8_t *)(&iqControl_1);		
	adata[1] = (*(uint8_t *)(&iqControl_1)+1);
	adata[2] = *(uint8_t *)(&iqControl_2);
	adata[3] = (*(uint8_t *)(&iqControl_2)+1);
	adata[4] = *(uint8_t *)(&iqControl_3);
	adata[5] = (*(uint8_t *)(&iqControl_3)+1);
	adata[6] = *(uint8_t *)(&iqControl_4);
	adata[7] = (*(uint8_t *)(&iqControl_4)+1);
}

void LK_Multi_set_RAM(uint8_t* adata,int32_t motoAngle){
	adata[0] = 0x95;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = *((uint8_t *)(&motoAngle));
	adata[5] = *((uint8_t *)(&motoAngle)+1);
	adata[6] = *((uint8_t *)(&motoAngle)+2);
	adata[7] = *((uint8_t *)(&motoAngle)+3);
}
void LK_Multi_set_ROM_noAdvise(uint8_t* adata){
	adata[0] = 0x19;		
	adata[1] = 0;
	adata[2] = 0;
	adata[3] = 0;
	adata[4] = 0;
	adata[5] = 0;
	adata[6] = 0;
	adata[7] = 0;
}

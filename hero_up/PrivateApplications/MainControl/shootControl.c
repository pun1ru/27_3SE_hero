/**
 * @file shootControl.c
 * @brief 射击控制实现：摩擦轮速度PID + 拨盘逻辑
 */

/* 注意：ShootControl 类型定义在 robot_control_task.h 中（通过 general_task_include.h 引入），
   shootControl.h 提供仅 extern 声明。二者不能同时定义类型，故在此不重复包含 shootControl.h。 */
#include "general_task_include.h"
#include "judge_receive.h"   // DataFromJudge

/* 单级摩擦轮模式：只给 LEFT、RIGHT、UP1 目标转速。 */
//#define SHOOT_SINGLE_STAGE_MODE

/* === 全局变量 === */
ShootControl shootControl={0};            //hxg
const ShootControl* _shootControl = &shootControl;

leastSquareLinear bulletSpeedAdaptation = {
	.x = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30},
	.count=0
};

float targetspeed[30]={0};
extern DataFromJudge bulletSpeed;
float predict_speed0;
float mardio_speed=15.75;
int16_t fric_speed_left_target , fric_speed_right_target,fric_speed_up_target;
int16_t fric_speed_left_target1 , fric_speed_right_target1,fric_speed_up_target1;
float current_fric_speed =3110;   // 吊射模式弹速; 3500,4650,dansu,4580-16.77//4785
float default_fric_speed = 3110;//常 规模式弹速
float front_fric_speed = 3210;//4550
float back_fric_speed = 4140;
float deltaspeed;
float emergesee;

extern void BulletSpeedReceive(void);

uint16_t CRC16_Modbus(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];   // 与当前字节异或

        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
uint8_t frame[8];

void ShootInputUpdate(void)
{

	fric_speed_left_target = shootControl.ShootTargetInput.fric_speed_rpm[LEFT];
	fric_speed_right_target = shootControl.ShootTargetInput.fric_speed_rpm[RIGHT];
	fric_speed_up_target = shootControl.ShootTargetInput.fric_speed_rpm[UP];
	fric_speed_left_target1 = shootControl.ShootTargetInput.fric_speed_rpm[LEFT1];
	fric_speed_right_target1 = shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1];
	fric_speed_up_target1 = shootControl.ShootTargetInput.fric_speed_rpm[UP1];

	/*---------------------------------------------------fric--------------------------------------------------------*/
	extern uint8_t shit_dan;
	if(shit_dan)
	{
	BulletSpeedReceive();
	shit_dan=0;
	}

	if(_robotState->fric_mode == FRIC_ON)
	{
		//注意符号
		if(_robotState->sniper==SNIPER_ON)
		{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] =+front_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] = -front_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[UP] = +back_fric_speed;
    	shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] =+back_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] = -back_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] = +front_fric_speed;
		}
		else
		{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] =+default_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] = -default_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[UP] = +default_fric_speed;
    	shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] =+default_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] = -default_fric_speed;
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] = +default_fric_speed;
		}
	}
	else if(_robotState->ctrl_terminal != CONTROL_STOP)                    //HXG1
	{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] = -200;///后右
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] = +200;//后左
		shootControl.ShootTargetInput.fric_speed_rpm[UP] = -200;//前下
    shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] = -200;//前右
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] = +200;//前左
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] = -200;//后上
	}
	else{
		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[UP] =0;
    shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] =0;
		shootControl.ShootTargetInput.fric_speed_rpm[UP1] =0;
	}

#ifdef SHOOT_SINGLE_STAGE_MODE
	shootControl.ShootTargetInput.fric_speed_rpm[UP] = 0.0f;
	shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] = 0.0f;
	shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] = 0.0f;
#endif

	/* 拨盘在hero_down，shoot_flag同步stir_mode状态（debug帧/上位机消费） */
	shootControl.ShootTargetInput.shoot_flag = (_robotState->stir_mode != STIR_LOCK) ? 1 : 0;
}

void ShootEstimateUpdate(void)
{
	/* 拨盘已移除，保留空壳 */
}

void ShootControlUpdate(void)
{
	//摩擦轮
	shootControl.ShootMotorControl.fric_target_output[LEFT] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[LEFT], \
																		shootControl.ShootTargetInput.fric_speed_rpm[LEFT] - _fricMotorRec[LEFT].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[RIGHT] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[RIGHT], \
																		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT] - _fricMotorRec[RIGHT].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[UP] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[UP], \
																		shootControl.ShootTargetInput.fric_speed_rpm[UP] - _fricMotorRec[UP].mechanical_speed_rpm);

    shootControl.ShootMotorControl.fric_target_output[LEFT1] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[LEFT1], \
																		shootControl.ShootTargetInput.fric_speed_rpm[LEFT1] - _fricMotorRec[LEFT1].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[RIGHT1] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[RIGHT1], \
																		shootControl.ShootTargetInput.fric_speed_rpm[RIGHT1] - _fricMotorRec[RIGHT1].mechanical_speed_rpm);
	shootControl.ShootMotorControl.fric_target_output[UP1] = PIDUpdate(&shootControl.ShootMotorControl.fric_speed_pid[UP1], \
																		shootControl.ShootTargetInput.fric_speed_rpm[UP1] - _fricMotorRec[UP1].mechanical_speed_rpm);
}

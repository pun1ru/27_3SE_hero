/**
 * @file music_mardio.c
 * @author 3SE 马丢
 * @brief 
 * @version 
 * @date 
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "music_mardio.h"

/**
 * @brief 
 * 
 * @param pr 
 * @param wait_time 
 * @param htim 
 */
void play_music(int pr, int wait_time, TIM_HandleTypeDef htim)
{
		if(pr>0){
    __HAL_TIM_PRESCALER(&htim, pr);
    __HAL_TIM_SetCompare(&htim, BUZZER_TIM_CHANNEL, 50);
		//DWT_Delay(wait_time);
    
			vTaskDelay(wait_time);
		}
		else{
		__HAL_TIM_PRESCALER(&htim, 0);
    __HAL_TIM_SetCompare(&htim, BUZZER_TIM_CHANNEL, 10);
		vTaskDelay(wait_time/60);}
}

/**
 * @brief 
 * 
 * @param note 
 * @return int 
 */
int from_notes_to_pr(float note)
{
    int pr;
    float hz;

    hz=261 * pow(2, (note -62) / 12.0);//利用十二平均律公式求出频率
    pr=(int) 168000000/hz/126000;//求出预分频器值
	
		if(note>150)
			pr=-1;//特殊标记
		
    return pr;
}

/**
 * @brief 
 * 
 * @param htim 
 * @param Channel 
 * @return int 
 */
int music_init(TIM_HandleTypeDef htim, uint32_t Channel)
{
    HAL_TIM_Base_Start(&htim);
    HAL_TIM_PWM_Start(&htim, BUZZER_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&htim, Channel);
    return 0;
}

/**
 * @brief 
 * 
 * @param notes 
 * @param size 
 * @param wait_time 
 * @param htim 
 */
void all_paly_music(float *notes, int size, int wait_time, TIM_HandleTypeDef htim)
{
    for(int i=0; i<size; i++)
    {
        play_music(from_notes_to_pr(notes[i]), wait_time, htim);//这里连续调用两个函数，第一个函数是将音符转换为PR值，第二个函数是播放PR值
    }
}

/**
* @brief 启动提示音伟大的舵轮神（不是舵轮王）封装的
 *
 */


/*用法实例：
在主函数中先初始化
 music_init(htim4,TIM_CHANNEL_3);//成功启动,初始化一切，自己改参数
然后
	int size=0;
		int wait_time=100;
			
		float mygo[]={16,16,16,16,16,16,16,16,14,14,14,14,12,12,12,12,12,12,12,12,14,14,14,14,16,16,16,16,16,17,17,16,16,16,16,14,14,14,14,14,14,100,100,100,100,100,100,100};//春日影
		wait_time=100;
		size=sizeof(mygo);
			
		float international[]={7,7,12,12,11,14,12,8,4,9.45,9.3};
		size=sizeof(international);
		wait_time=100;
		
		float BigRockBrokeHeart_1[]=
{10,12,14,10,12,8,10,12,14,10,12,8,10,12,14,100,
 10,12,8,10,12,14,10,12,8,10,12,14,10,12,8,100};

		size = sizeof(BigRockBrokeHeart_1) / sizeof(BigRockBrokeHeart_1[0]);
    float BigRockBrokeHeart_2[size];
		for(int i=0;i<=size;i++){
			BigRockBrokeHeart_2[i]=BigRockBrokeHeart_1[i]+4;
		}
     wait_time=170;
		
		
		all_paly_music(BigRockBrokeHeart_1,size,wait_time,htim4);
    all_paly_music(BigRockBrokeHeart_1,size,wait_time,htim4);
		all_paly_music(BigRockBrokeHeart_2,size,wait_time,htim4);
    all_paly_music(BigRockBrokeHeart_2,size,wait_time,htim4);
*/
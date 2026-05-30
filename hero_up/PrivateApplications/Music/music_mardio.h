/**
 * @file music_mardio.h
 * @author 3SE 马丢
 * @brief 
 * @version 
 * @date 
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _MUSIC_MARDIO_H_
#define _MUSIC_MARDIO_H_

#include "general_task_include.h"

void play_music(int pr,int wait_time,TIM_HandleTypeDef htim);
int from_notes_to_pr(float note);
int music_init(TIM_HandleTypeDef htim,uint32_t Channel);
void all_paly_music(float *notes,int size,int wait_time,TIM_HandleTypeDef htim);

#endif

#ifndef _MUSIC_KU
#define _MUSIC_KU
	static const float famima[] = {17, 13, 8, 13, 15, 20, 20, 15, 17, 15, 8, 13, 13, 13, 13};
	//uint16_t time1[] = {100, 100, 100, 100, 100, 100, 200, 100, 100, 100, 100, 100, 100, 100, 100};
	static const float umbrella[] = {21, 21, 21, 19, 21, 21, 21, 14, 16, 19, 14, 11, 9, 9, 9};
	static const float mygo[]={16,16,16,16,14,14,12,12,12,12,14,14,16,16,16,17,16,16,14,14,14,100,100,100,100};//春日影 50
	static const float battlefiled[]={10,10,10,10,10,10,10,10,10,10,100,100,100,100,12,12,8,8,5,5,10,10,10,10,10,10,10,10,10,10,100,100,100,100,\
												13,12,10,10,10,10,12,13,15,15,15,15,13,12,13,13,13,13,12,10,12,12,8,8,5,5,10,10,10,10,10,10,10,10,10};//战地 150
	static const float bleach[]={2,2,201,3,3,201,4,4,\
		  5,5,201,5,5,80,80,5,5,5,5,101,5,5,5,5,101,5,5,5,5,				4,4,4,4,201,4,4,4,4,4,4,\
	80,80,3,3,201,3,3,80,80,3,3,3,3,101,3,3,3,3,101,3,3,3,3,				2,2,2,2,201,2,2,2,2,2,2,\
	80,80,5,5,201,5,5,80,80,5,5,5,5,101,8,8,8,8,101,7.2,7.2,7.2,7.2,4,4,4,4,101,4,4,4,4,7.2,7.2,7.2,7.2,\
	80,80,3,3,101,3,3,80,80,3,3,3,3,101,5,5,5,5,7,7,7,5,5,5,5,5,5,5,5,};//-8 100
	static const float alone_earth[]={
											 15,14,12,14,80,3,5,7,80,3,80,2,2,80,\
											 15,14,12,10,80,3,5,7,80,3,80,10,10,80,\
											 15,14,12,14,80,3,5,7,80,3,80,2,2,80,\
											 17,17,19,19,12,12,15,17,17,19,19,22,22,19,19,80,\
											 24,24,22,22,17,19,80,19,80,12,80,10,80,12,12,12
											};//180
	static const float run_to_night[]={};
	static const float out_wild[]={-1,-1,4,4,8,8,80,4,9,8,6,4,6,8,4,80,\
										-1,-1,4,4,8,8,80,4,9,8,6,4,6,8,11,8,80,\
										};//200
	static const float sea_bottom[]={};
#endif
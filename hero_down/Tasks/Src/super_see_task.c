#include "super_see_task.h"
extern SemaphoreHandle_t superSeeSign;
extern int com_complet;
void SuperSeeTask(void* argument)
{
	static uint32_t last_tick_count, current_tick_count, this_tick_count = 0;
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._super_see_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._super_see_task = &this_tick_count;
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	while(1)
	{		
		xSemaphoreTake(superSeeSign, portMAX_DELAY);//信号量阻塞
		all_distance_see();
		com_complet=1;
		vTaskDelayUntil(&current_tick_count, MUSIC_TASK_PERIOD_SET);//对于阻塞任务应该不需要这个
	}
}
/*吊射主要调参 遇事不决，改CD*/
float small_height=0.17896;
float cd=0.5;
int com_complet=1;
double distance_check_angle=0;
void all_distance_see(){
	float speed=_bulletSpeed->predict_speed;
	if(speed==0)
		speed=TARGET_BULLET_SPEED;
	//aacheck=simpleParabolicCheck(15.6, distance_check.distance_check_translate.distance_select*cos(_gimbalControl->GimbalEstimate.small_pitch_actual_angle*PI/180),  distance_check.distance_check_translate.distance_select*sin(_gimbalControl->GimbalEstimate.small_pitch_actual_angle*PI/180)+0.131);
	if(_robotState->follow==FOLLOW_ON /*&& follow_leap_flag==1*/ &&\
		 simpleParabolicCheck(speed, distance_check.distance_check_translate.distance_select*cos(_gimbalControl->GimbalEstimate.small_pitch_actual_angle*PI/180),  distance_check.distance_check_translate.distance_select*sin(_gimbalControl->GimbalEstimate.small_pitch_actual_angle*PI/180)+small_height) && distance_check.distance_check_translate.distance_select!=0)
				{
				//aacheck++;
				distance_check.delay=0;//微小史，已废置
				distance_check_angle=\
					findAngleForHeight(speed,\
					distance_check.distance_check_translate.distance_select*cos(_gimbalControl->GimbalEstimate.small_pitch_actual_angle*PI/180),\
					distance_check.distance_check_translate.distance_select*sin(_gimbalControl->GimbalEstimate.small_pitch_actual_angle*PI/180)+small_height,\
				0.0445,  (42.5 / 2) / 1000,cd, 0.0005);				
//				if(distance_check_angle<5||distance_check_angle>35){//异常数据
//					distance_check.distance_check_translate.angle=24.0;
//				}
				//if(distance_check_angle<35&&distance_check_angle>5)
					//__enable_irq ();
					//{
						distance_check.distance_check_translate.angle=distance_check_angle;	
					//}		
				//
				
				//follow_leap_flag=0;
					//__enable_irq ();//请不要把这个给那个了
					}
				else{
					distance_check.distance_check_translate.angle=24.0;
				}
}
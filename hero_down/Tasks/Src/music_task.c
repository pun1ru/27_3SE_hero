#include "music_mardio.h"
static void StartupNotice(const float* puzi,int wait_time,float pintiao);
extern QueueHandle_t g_musicQueue;
void Error_check();
void Error_fuck();
CircuitMonitor circuitMonitror={0};
void MusicTask(void* argument)
{
	static uint32_t last_tick_count, current_tick_count, this_tick_count = 0;
	static uint16_t task_counter;
	_taskMonitor->TaskFrameCounterPtr._music_task = &task_counter;
	_taskMonitor->TaskRunPeriodPtr._music_task = &this_tick_count;
	
	current_tick_count = last_tick_count = xTaskGetTickCount();	
	while(1)
	{		
		/*轮询主动通讯检测*/
		 Error_check();
		 
		 short music_receive,a;
		 /*错误处理音效*/
		 if (xQueueReceive(g_musicQueue, &music_receive, 0) == pdPASS){
				switch(music_receive){
				 case 1:
					 StartupNotice(umbrella,200,-8);//
				 break;
				 case 2:
					 StartupNotice(battlefiled,150,-8);//CAN报错音乐
				 break;
				 case 3:
					 StartupNotice(alone_earth,180,-8);
				 break;
				 default:
					 a=music_receive;//call of stack
					 StartupNotice(bleach,100,-8);
			 }
		 }
		//xPortGetFreeHeapSize();
		/*计算任务实际运行周期*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
		vTaskDelayUntil(&current_tick_count, MUSIC_TASK_PERIOD_SET);
	}
}
static void StartupNotice(const float* puzi,int wait_time,float pintiao)
{
	music_init(BUZZER_TIM, BUZZER_TIM_CHANNEL);
	int size = 0;
		
	#define playThis puzi
	//float touhou[] = {3, 3, 8, 8, 10, 10, 13, 13, 10, 10, 8, 8, 3, 8, 10, 10, 3, 3, 8, 8, 10, 10, 13, 13, 15, 15, 10, 10, 8, 10, 8, 6};
	//float zuki[] = {15, 15, 17, 17, 20, 22, 17, 15, 17, 17, 12, 1, 12, 1, 12, 8, 10, 10, 5, 8, 10, 13, 15, 13, 15, 1000, 15, 13, 12, 20, 17, 17};
	//bleach 死神 number one
	size = sizeof(out_wild) / sizeof(float);
	for (int i = 0; i < size; i++)
	{
		play_music(from_notes_to_pr(puzi[i]+pintiao), wait_time, BUZZER_TIM);
		//HAL_IWDG_Refresh(&hiwdg1);
	}
//	HAL_Delay(10);
	__HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0);
}
extern const DMJ4310MotorRec* _DMyawMotorRec;
extern const DMJ4310MotorRec* _jointMotorEec;
extern const DMJ4310MotorRec* _caterpillarMotorRec;
void Error_check()
{
	// =================================================================
	//                       第一部分：外设错误检查
	// =================================================================
	
	// 1. 检查4个底盘电机 (Chassis Motors) — CAN fdcan2
	for (int i = 0; i < 4; i++)
	{
		if(circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[i] == _chassisMotorRec[i].frame_counter)
			circuitMonitror.ifCircuitError.chassisMotorError[i] = 1;
		else
			circuitMonitror.ifCircuitError.chassisMotorError[i] = 0;
	}
	
	// 2. 检查云台Yaw DM电机 — CAN fdcan1
	if(circuitMonitror.CircuitCounterPtr.yawMotor_frame_counter == _DMyawMotorRec->frame_counter)
		circuitMonitror.ifCircuitError.yawMotorError = 1;
	else
		circuitMonitror.ifCircuitError.yawMotorError = 0;

	// 3. 检查4个关节电机 (Joint Motors) — CAN fdcan1
	for (int i = 0; i < 4; i++)
	{
		if(circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[i] == _jointMotorEec[i].frame_counter)
			circuitMonitror.ifCircuitError.jointMotorError[i] = 1;
		else
			circuitMonitror.ifCircuitError.jointMotorError[i] = 0;
	}

	// 4. 检查2个履带电机 (Caterpillar Motors) — CAN fdcan2
	for (int i = 0; i < 2; i++)
	{
		if(circuitMonitror.CircuitCounterPtr.caterpillarMotor_frame_counter[i] == _caterpillarMotorRec[i].frame_counter)
			circuitMonitror.ifCircuitError.caterpillarMotorError[i] = 1;
		else
			circuitMonitror.ifCircuitError.caterpillarMotorError[i] = 0;
	}

	// 5. 检查拨弹电机 (Stir Motor) — CAN fdcan1
	if(circuitMonitror.CircuitCounterPtr.stirMotor_frame_counter == _stirMotorRec->frame_counter)
		circuitMonitror.ifCircuitError.stirMotorError = 1;
	else
		circuitMonitror.ifCircuitError.stirMotorError = 0;

	// 6. 检查超级电容 (Super Capacitor) — CAN fdcan2/fdcan3
	if(circuitMonitror.CircuitCounterPtr.superCapacity_frame_counter == _superCapacity->frame_counter)
		circuitMonitror.ifCircuitError.superCapacityError = 1;
	else
		circuitMonitror.ifCircuitError.superCapacityError = 0;


	// =================================================================
	//       第二部分：更新所有“上次帧计数值”，为下一轮检查做准备
	// =================================================================

	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[0] = _chassisMotorRec[0].frame_counter;
	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[1] = _chassisMotorRec[1].frame_counter;
	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[2] = _chassisMotorRec[2].frame_counter;
	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[3] = _chassisMotorRec[3].frame_counter;
	
	circuitMonitror.CircuitCounterPtr.yawMotor_frame_counter = _DMyawMotorRec->frame_counter;
	
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[0] = _jointMotorEec[0].frame_counter;
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[1] = _jointMotorEec[1].frame_counter;
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[2] = _jointMotorEec[2].frame_counter;
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[3] = _jointMotorEec[3].frame_counter;
	
	circuitMonitror.CircuitCounterPtr.caterpillarMotor_frame_counter[0] = _caterpillarMotorRec[0].frame_counter;
	circuitMonitror.CircuitCounterPtr.caterpillarMotor_frame_counter[1] = _caterpillarMotorRec[1].frame_counter;
	
	circuitMonitror.CircuitCounterPtr.stirMotor_frame_counter = _stirMotorRec->frame_counter;
	circuitMonitror.CircuitCounterPtr.superCapacity_frame_counter = _superCapacity->frame_counter;
}
/*用来看爆栈*/
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	
	xTaskHandle xTask1=xTask;
	signed char pcTaskName1=*pcTaskName;
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}

void CanFix(){
	static uint8_t count=0;
	count++;
	//uint8_t now=count%3;
	switch(count%100){
		case 1:
			if(hfdcan1.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan1);
		case 2:
			if(hfdcan2.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan2);
		case 3:
			if(hfdcan1.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan3);
	}
 }
void Error_fuck(){
//	 if(circuitMonitror.ifCircuitError.smallpitchMotorError||circuitMonitror.ifCircuitError.yawMotorError)
		 CanFix();
}
#include "music_mardio.h"
#include "general_task_include.h"
#include "general_define.h"
static void StartupNotice(const float* puzi, int arr_size, int wait_time, float pintiao);
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
		/*轮询通讯状态检测*/
		 Error_check();
		 
		 short music_receive,a;
			 /*蜂鸣器播放有效*/
		 if (xQueueReceive(g_musicQueue, &music_receive, 0) == pdPASS){
				switch(music_receive){
				 case 1:
					 //StartupNotice(my_most_precious_treasure, sizeof(my_most_precious_treasure)/sizeof(float), 150, -8);//
				 break;
				 case 2:
					 //StartupNotice(battlefiled, sizeof(battlefiled)/sizeof(float), 150, -8);//CAN错误报警
				 break;
				 case 3:
					 StartupNotice(alone_earth, sizeof(alone_earth)/sizeof(float), 180, -8);
				 break;
				 default:
					 a=music_receive;//call of stack
					 StartupNotice(bleach, sizeof(bleach)/sizeof(float), 100, -8);
			 }
		 }
		//xPortGetFreeHeapSize();
		/*任务循环计数更新*/
		task_counter++;
		current_tick_count = xTaskGetTickCount();
		this_tick_count = current_tick_count - last_tick_count;
		last_tick_count = current_tick_count;
		
		vTaskDelayUntil(&current_tick_count, MUSIC_TASK_PERIOD_SET);
	}
}
static void StartupNotice(const float* puzi, int arr_size, int wait_time, float pintiao)
{
	music_init(BUZZER_TIM, BUZZER_TIM_CHANNEL);
	for (int i = 0; i < arr_size; i++)
	{
		play_music(from_notes_to_pr(puzi[i] + pintiao), wait_time, BUZZER_TIM);
	}
	__HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0);
}
extern const DMJ4310MotorRec* _jointMotorEec;
extern const DMJ4310MotorRec* _caterpillarMotorRec;
void Error_check()
{
	// =================================================================
	//                       第一部分：检查数据更新
	// =================================================================
	
	// 1. 检查4个底盘电机 (Chassis Motors) 用 CAN fdcan2
	for (int i = 0; i < 4; i++)
	{
		if(circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[i] == _chassisMotorRec[i].frame_counter)
			circuitMonitror.ifCircuitError.chassisMotorError[i] = 1;
		else
			circuitMonitror.ifCircuitError.chassisMotorError[i] = 0;
	}
	
	// 2. 检查云台Yaw DM电机 用 CAN fdcan1
		circuitMonitror.ifCircuitError.yawMotorError = 1;
	else
		circuitMonitror.ifCircuitError.yawMotorError = 0;

	// 3. 检查4个关节电机 (Joint Motors) 用 CAN fdcan1
	for (int i = 0; i < 4; i++)
	{
		if(circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[i] == _jointMotorEec[i].frame_counter)
			circuitMonitror.ifCircuitError.jointMotorError[i] = 1;
		else
			circuitMonitror.ifCircuitError.jointMotorError[i] = 0;
	}

	// 4. 检查2个履带电机 (Caterpillar Motors) 用 CAN fdcan2
	for (int i = 0; i < 2; i++)
	{
		if(circuitMonitror.CircuitCounterPtr.caterpillarMotor_frame_counter[i] == _caterpillarMotorRec[i].frame_counter)
			circuitMonitror.ifCircuitError.caterpillarMotorError[i] = 1;
		else
			circuitMonitror.ifCircuitError.caterpillarMotorError[i] = 0;
	}

	// 5. 检查拨弹电机 (Stir Motor) 用 CAN fdcan1
	if(circuitMonitror.CircuitCounterPtr.stirMotor_frame_counter == _stirMotorRec->frame_counter)
		circuitMonitror.ifCircuitError.stirMotorError = 1;
	else
		circuitMonitror.ifCircuitError.stirMotorError = 0;

	// 6. 检查超级电容 (Super Capacitor) 用 CAN fdcan2/fdcan3
	if(circuitMonitror.CircuitCounterPtr.superCapacity_frame_counter == _superCapacity->frame_counter)
		circuitMonitror.ifCircuitError.superCapacityError = 1;
	else
		circuitMonitror.ifCircuitError.superCapacityError = 0;


	// =================================================================
	//      第二部分：保存当前帧计数值作为下一轮比较基准
	// =================================================================

	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[0] = _chassisMotorRec[0].frame_counter;
	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[1] = _chassisMotorRec[1].frame_counter;
	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[2] = _chassisMotorRec[2].frame_counter;
	circuitMonitror.CircuitCounterPtr.chassisMotor_frame_counter[3] = _chassisMotorRec[3].frame_counter;
	
	
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[0] = _jointMotorEec[0].frame_counter;
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[1] = _jointMotorEec[1].frame_counter;
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[2] = _jointMotorEec[2].frame_counter;
	circuitMonitror.CircuitCounterPtr.jointMotor_frame_counter[3] = _jointMotorEec[3].frame_counter;
	
	circuitMonitror.CircuitCounterPtr.caterpillarMotor_frame_counter[0] = _caterpillarMotorRec[0].frame_counter;
	circuitMonitror.CircuitCounterPtr.caterpillarMotor_frame_counter[1] = _caterpillarMotorRec[1].frame_counter;
	
	circuitMonitror.CircuitCounterPtr.stirMotor_frame_counter = _stirMotorRec->frame_counter;
	circuitMonitror.CircuitCounterPtr.superCapacity_frame_counter = _superCapacity->frame_counter;
}
/*栈溢出回调*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
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
				break;
		case 2:
			if(hfdcan2.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan2);
				break;
		case 3:
			if(hfdcan3.ErrorCode!=0x00000000)
				HAL_FDCAN_ErrorCallback(&hfdcan3);
				break;
	}
 }
void Error_fuck(){
//	 if(circuitMonitror.ifCircuitError.smallpitchMotorError||circuitMonitror.ifCircuitError.yawMotorError)
		 CanFix();
}
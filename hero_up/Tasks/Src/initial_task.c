/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */



#include "general_define.h"
#include "tim.h"
#include "usart.h"

#include "general_task_include.h"
#include "bsp_dwt.h"

#include "ws2812.h"
#include "music_mardio.h"

TaskHandle_t decisionTaskHandle;
TaskHandle_t stateMachineTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t upperPCCommTaskHandle;
TaskHandle_t remoteRecTaskHandle;
TaskHandle_t uiOperationTaskHandle;
TaskHandle_t monitorTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t estimateTaskHandle;
TaskHandle_t debugTaskHandle;
TaskHandle_t musicTaskHandle;
QueueHandle_t g_musicQueue;//报错音乐的队列

static void StartupNotice();
/*外设及freertos相关初始化所需的其他源文件中的全局变量*/
void InitTask(void const * argument)
{
	taskENTER_CRITICAL();
	
	// 启动，WS2812变红
	WS2812_SPI_Ctrl(50, 0, 0); //66CCFF
	
	DWT_Init(480);
	/*初始化PWM，让蜂鸣器先狗叫*/
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
	WS2812_PWM_Init();  // WS2812 PWM+DMA LED strip init (replaces servo PWM on PA0/TIM2_CH1)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);//升温
	
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,500);//蜂鸣器！我的心跳！
	
	xTaskCreate(MonitorTask,        "MonitorTask_",       128,  NULL,  7,  &monitorTaskHandle      );
	xTaskCreate(StateMachineTask,   "StateMachineTask_",  2048, NULL,  7,  &stateMachineTaskHandle );	
	xTaskCreate(DecisionTask, 	    "DecisionTask_", 	    512,  NULL,  5,  &decisionTaskHandle     );
	xTaskCreate(EstimateTask,       "EstimateTask_",      512,  NULL,  5,  &estimateTaskHandle     );
	xTaskCreate(ControlTask,        "ControlTask_",       512,  NULL,  5,  &controlTaskHandle      );
	xTaskCreate(IMUTask, 						"IMUTask_",						512,  NULL,	 5,	 &imuTaskHandle			 		 );
	xTaskCreate(DebugTask,          "DebugTask_",         256,  NULL,  4,  &debugTaskHandle        );
	xTaskCreate(UpperPCCommTask,  	"UpperPCCommTask_", 	256,  NULL,  5,  &upperPCCommTaskHandle	 );	
	xTaskCreate(MusicTask,					"MusicTaskHandle_",   512,	NULL,  2,  &musicTaskHandle				 );
	//消息队列
	g_musicQueue=xQueueCreate(1,2);
	// 任务创建结束，进入除了PWM之外的外设初始化，WS2812变黄
	WS2812_SPI_Ctrl(25, 25, 0);
	
	/* Enable 5V给舵机用 */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	PeripheralRecEnable();
	// 完全OK，WS2812变绿
	WS2812_SPI_Ctrl(0, 10, 0);
	 

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);//关掉蜂鸣器
		   	
	vTaskDelete(NULL);   
	taskEXIT_CRITICAL();   
}



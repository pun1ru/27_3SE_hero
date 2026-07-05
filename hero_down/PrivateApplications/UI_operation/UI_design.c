#include "UI_design.h"
#include "general_task_include.h"
#include <stdio.h>

/* RS485接收的云台数据 */
extern volatile float gimbal_yaw_target_rx_d;
extern volatile float gimbal_pitch_target_rx_d;
extern volatile float gimbal_pitch_rx_d;
/*---------------------------------------------------UI region-------------------------------------------*/
UIframe_t UIframe;
UIframe_t* _UIframe = &UIframe;

void idObtain(uint16_t* receiver_id, uint16_t* sender_id); void UiOperation();
void UiInit(uint32_t const * ui);
void FrameUpdate(uint32_t * ui);

void idObtain(uint16_t* receiver_id, uint16_t* sender_id)
{
	if(ext_game_robot_status.robot_id != 0)
		{
			*sender_id= ext_game_robot_status.robot_id; //发送方id
			
			if(ext_game_robot_status.robot_id < 100)
				*receiver_id = (ext_game_robot_status.robot_id | 0x0100);
			else
				*receiver_id = ext_game_robot_status.robot_id + 256;
		}
}

/**
 * @brief  ui绘制函数
 * @note   
 * @param  none
 * @retval None
 */
void UiOperation()
{
	static uint32_t ui = 0;
	static uint8_t InitFlag = 0;	
	if(ui %200 < 12)
		UiInit(&ui);
	else
		FrameUpdate(&ui);		
	ui++;

}



/**
* @brief  ui初始化，操作类型为OperateAdd
 * @note   除字符类型外，其他类型均把全部图形画完再发送，字符类型画一个发一个
 * @param  none
 * @retval None
 */
int delta=27;
void UiInit(uint32_t const * ui)
{
	uint8_t* uname;//uint8_t数组，必须三字节
	uint16_t subcontent_id = 0;
	uint8_t TransmitOk = 0;
	
	uint16_t sender_id, receiver_id;
	idObtain(&receiver_id, &sender_id);
	
	uint8_t index = *ui % 12;
	switch(index)
	{
		case 0:
		{
			//图层六，云台目标/实际角度 + aim检测（合并一帧）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_gim;
			char gim_text[48];
			const char* aim_status = ((fabs((double)gimbal_yaw_target_rx_d) > 0.01) && (fabs((double)gimbal_pitch_target_rx_d) > 0.01)) ? "OK" : "Ohno";
			
			uname = (unsigned char*)"gim";
			sprintf(gim_text, "TY:%.1f Y:%.1f\nTP:%.1f P:%.1f\n%s",
				(double)gimbal_yaw_target_rx_d,
				(double)_gimbalControl->GimbalEstimate.yaw_angle_d,
				(double)gimbal_pitch_target_rx_d,
				(double)gimbal_pitch_rx_d,
				aim_status);
			DrawChar(&char_gim, OperateAdd, 1550, 600, uname, 2, 20, 6, (uint8_t*)gim_text, (uint8_t)strlen(gim_text), ColorYellow);
			
			CharacterToUIframe(&char_gim, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;   
		}
		
		case 1:
		{
			//图层三
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3 = {};
			
			//电容电压值
			uname = (unsigned char*)"cvo";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					OperateAdd, 980, 200, uname, 2, 20, 3, _superCapacity->cap_volt * 1000, ColorGreen);
			
			//激光测距值
			uname = (unsigned char*)"dos";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[2],\
					OperateAdd, 980, 830, uname, 2, 20, 3, _distance_check->distance_check_translate.distance_select * 1000, ColorGreen);
			
			
			//前哨站准星 我也不知道，
				uname = (unsigned char*)"qia";
				DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
						 OperateAdd, 960, 900, 960, 200, uname, 1, 3, ColorCyan);
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 2:
		{
			//图层六，鼠标锁定状态（sniper下pitch/yaw锁定）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			ext_interaction_character_t char1;
			if(_robotState->mouse_fix == MOUSE_FIX_ON)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateAdd, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:on ", 13, ColorAmaranth);
			}
			else if(_robotState->mouse_fix == MOUSE_FIX_OFF)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateAdd, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:off", 13, ColorGreen);
			}
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
		
			break;
		}
		
		case 3:
		{
			//图层六，sniper模式显示（STAIR上方）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_snp;
			if(_robotState->sniper == SNIPER_ON)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateAdd, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:ON ", 10, ColorAmaranth);
			}
			else if(_robotState->sniper == SNIPER_OFF)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateAdd, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:OFF", 10, ColorGreen);
			}
			
			CharacterToUIframe(&char_snp, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
		
			break;
		}
		
		case 4:
		{
			//图层六，上台阶状态（边缘显示，不遮挡主视角）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char2;
			
			uname = (unsigned char*)"std";
			switch(_robotState->stand_mode)
			{
				case ROBOT_STAND_MODE_NORMAL:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:NORMAL    ", 16, ColorGreen);
					break;
				case ROBOT_STAND_MODE_PRE_STAIR:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE       ", 16, ColorYellow);
					break;
				case ROBOT_STAND_MODE_STAIR_UP:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:UP        ", 16, ColorAmaranth);
					break;
				case ROBOT_STAND_MODE_PRE_DOWN_STAIR:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE_DOWN  ", 16, ColorOrange);
					break;
				default:
					DrawChar(&char2, OperateAdd, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:???       ", 16, ColorOrange);
					break;
			}
			
			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			break;
		}
		
		case 5:
		{
			//图层六，电容电压
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			ext_interaction_character_t char1;
			
			uname = (unsigned char *)"cav";
			DrawChar(&char1, OperateAdd, 800, 200, uname, 2, 20, 6, (uint8_t*)"CapVot:   ", 10, ColorYellow);
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		
		case 6:
		{
			//图层六
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3 = {};
			//准星1
			uname = (unsigned char*)"ap1";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					 OperateAdd, 850-delta,  540, 970-delta, 540, uname, 1, 6, ColorGreen);
			//清除已删除的旧图形残留（激光落点、自动补偿条）
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
					 OperateDelete, 0, 0, 0, 0, (uint8_t*)"hen", 1, 6, ColorYellow);
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[3],\
					 OperateDelete, 0, 0, 0, 0, (uint8_t*)"shu", 1, 6, ColorAmaranth);
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[4],\
					 OperateDelete, 0, 0, 0, 0, (uint8_t*)"ap3", 1, 6, ColorGreen);
				
			//准星2
//			uname = (unsigned char*)"ap2";
//			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
//					 OperateAdd, 850-delta, 432, 970-delta, 432, uname, 1, 6, ColorGreen);//
			
			//电容条
			uint8_t color;
			
			if(_superCapacity->cap_volt < 12)
				color = ColorOrange;
			else if(_superCapacity->cap_volt < 18)
				color = ColorYellow;
			else 
				color = ColorGreen;
			
			uint32_t Power_Line = (uint32_t)(((_superCapacity->cap_volt - 12) / 12.0f) * 600); //电容条
			
			uname = (unsigned char *)"cap";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[2],\
					 OperateAdd, 700, 100, (700 + Power_Line), 100, uname, 40, 6, color);
			
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 7:
		{//吊射模式显示
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"dio";
				if(_robotState->follow==FOLLOW_ON)//吊射模式
				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
			else if(_robotState->follow==FOLLOW_OFF)//普通模式
				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off ", 10, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
		    break;
		}
		case 8:
		{
			//图层七，摩擦轮开关标志
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char1;
			
			uname = (unsigned char*)"fri";
			if(_robotState->fric_mode == FRIC_ON)
				DrawChar(&char1, OperateAdd, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: On  ", 10, ColorAmaranth);
			else
				DrawChar(&char1, OperateAdd, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: Close", 11, ColorGreen);
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		case 9:
		{
			//图层九

			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3 = {};
			
			uname = (unsigned char*)"aaa";
			DrawRect(&figure3.ext_interaction_figure_3.interaction_figure[0],\
			OperateAdd, 685, 240, 1235, 710, uname, 5, 8, ColorAmaranth);
			
			static uint8_t draw_flag = 0;
			if(_robotState->chassis_mode == CHASSIS_SEPARATE && !draw_flag)
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateAdd, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateAdd, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 1;
		    }
			else if( _robotState->chassis_mode != CHASSIS_SEPARATE && draw_flag )
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateDelete, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateDelete, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 0;
			}
			
			uname = (unsigned char*)"fff";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[3],\
			         OperateAdd, 910-delta, 540, 910-delta, 380, uname, 1, 8, ColorGreen);
			
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		
		case 10:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"tuy";
			if(_robotState->capacity_mode == NO_CAPACITY)
				DrawChar(&char3, OperateAdd, 200, 730, uname, 2, 20, 6, (uint8_t*)"LOW POWER", 9, ColorAmaranth);
			else if(_robotState->capacity_mode == CAPACITY)
				DrawChar(&char3, OperateAdd, 200, 730, uname, 2, 20, 6, (uint8_t*)"NORMAL POWER", 12, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 11:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"fuk";
			if(_robotState->chassis_mode == CHASSIS_REVOLVE)
				DrawChar(&char3, OperateAdd, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin On", 7, ColorAmaranth);
			else
				DrawChar(&char3, OperateAdd, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin Off", 8, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
//			
			break;
		}
		
//		case 11:
//			{//吊射模式显示
//			subcontent_id = 0x0110;
//			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
//			ext_interaction_character_t char3;
//			
//			uname = (unsigned char*)"dio";
//				if(_robotState->follow==FOLLOW_ON)//吊射模式
//				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
//			else if(_robotState->follow==FOLLOW_OFF)//普通模式
//				DrawChar(&char3, OperateAdd, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off ", 10, ColorGreen);
//			
//			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
//			UIframeTransmit(_UIframe->data, subcontent_id);
//			UIframeClear(_UIframe->data);
//			
//			break;
//		}
//		
//		case 12:
//		{
////			subcontent_id = 0x0110;
////			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
////			ext_interaction_character_t char2;
////			
////			uname = (unsigned char*)"dis";
////			DrawChar(&char2, OperateAdd, 1080, 800, uname, 2, 20, 6, (uint8_t*)"DISTANCE:    	", 10, ColorYellow);
////			
////			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
////			UIframeTransmit(_UIframe->data, subcontent_id);
////			UIframeClear(_UIframe->data);
//		
//			break;
//		}
			
		default:
			break;
	}
	
}


/**
* @brief  ui更新，操作类型为OperateChange
 * @note   除字符类型外，其他类型均把全部图形画完再发送，字符类型画一个发一个
 * @param  none
 * @retval None
 */
void FrameUpdate(uint32_t * ui)
{
	uint8_t* uname;//uint8_t数组，必须三字节
	uint16_t subcontent_id = 0;
	uint8_t TransmitOk = 0;
	
	uint16_t sender_id, receiver_id;
	idObtain(&receiver_id, &sender_id);
	
	uint8_t index = *ui % 12;
	switch(index)
	{
		case 0:
		{
			//图层六，云台目标/实际角度 + aim检测（合并一帧）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_gim;
			char gim_text[48];
			const char* aim_status = ((fabs((double)gimbal_yaw_target_rx_d) > 0.01) && (fabs((double)gimbal_pitch_target_rx_d) > 0.01)) ? "OK" : "Ohno";
			
			uname = (unsigned char*)"gim";
			sprintf(gim_text, "TY:%.1f Y:%.1f\nTP:%.1f P:%.1f\n%s",
				(double)gimbal_yaw_target_rx_d,
				(double)_gimbalControl->GimbalEstimate.yaw_angle_d,
				(double)gimbal_pitch_target_rx_d,
				(double)gimbal_pitch_rx_d,
				aim_status);
			DrawChar(&char_gim, OperateChange, 1550, 600, uname, 2, 20, 6, (uint8_t*)gim_text, (uint8_t)strlen(gim_text), ColorYellow);
			
			CharacterToUIframe(&char_gim, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 1:
		{
			//图层三
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3= {};
			
			//电容电压值
			uname = (unsigned char*)"cvo";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					OperateChange, 980, 200, uname, 2, 20, 3, _superCapacity->cap_volt * 1000, ColorGreen);
			
			//激光测距值
			uname = (unsigned char*)"dos";
			DrawFloat(&figure3.ext_interaction_figure_3.interaction_figure[3],\
					OperateChange, 980, 830, uname, 2, 20, 3, _distance_check->distance_check_translate.distance_select * 1000, ColorGreen);
			
			//前哨站准星
				uname = (unsigned char*)"qia";
				DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
						 OperateChange, 960, 900, 960, 200, uname, 1, 3, ColorCyan);
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 2:
		{
			//图层六，鼠标锁定状态
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			ext_interaction_character_t char1;
			if(_robotState->mouse_fix == MOUSE_FIX_ON)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateChange, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:on ", 13, ColorAmaranth);
			}
			else if(_robotState->mouse_fix == MOUSE_FIX_OFF)
			{
				uname = (unsigned char*)"mfx";
				DrawChar(&char1, OperateChange, 200, 630, uname, 2, 20, 6, (uint8_t*)"MOUSE_FIX:off", 13, ColorGreen);
			}
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 3:
		{
			//图层六，sniper模式显示（STAIR上方）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char_snp;
			if(_robotState->sniper == SNIPER_ON)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateChange, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:ON ", 10, ColorAmaranth);
			}
			else if(_robotState->sniper == SNIPER_OFF)
			{
				uname = (unsigned char*)"snp";
				DrawChar(&char_snp, OperateChange, 200, 870, uname, 2, 20, 6, (uint8_t*)"SNIPER:OFF", 10, ColorGreen);
			}
			
			CharacterToUIframe(&char_snp, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 4:
		{
			//图层六，上台阶状态（边缘显示，不遮挡主视角）
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char2;
			
			uname = (unsigned char*)"std";
			switch(_robotState->stand_mode)
			{
				case ROBOT_STAND_MODE_NORMAL:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:NORMAL    ", 16, ColorGreen);
					break;
				case ROBOT_STAND_MODE_PRE_STAIR:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE       ", 16, ColorYellow);
					break;
				case ROBOT_STAND_MODE_STAIR_UP:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:UP        ", 16, ColorAmaranth);
					break;
				case ROBOT_STAND_MODE_PRE_DOWN_STAIR:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:PRE_DOWN  ", 16, ColorOrange);
					break;
				default:
					DrawChar(&char2, OperateChange, 200, 820, uname, 2, 20, 6, (uint8_t*)"STAIR:???       ", 16, ColorOrange);
					break;
			}
			
			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			break;
		}
		
		case 5:
		{
			//图层六，电容电压
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char1;
			
			uname = (unsigned char *)"cav";
			DrawChar(&char1, OperateChange, 800, 200, uname, 2, 20, 6, (uint8_t*)"CapVot:   ", 10, ColorYellow);
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 6:
		{
			//图层六
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3= {};
			//准星1
			uname = (unsigned char*)"ap1";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[0],\
					 OperateChange, 850-delta, 540, 970-delta, 540, uname, 1, 6, ColorGreen);
				
//			uname = (unsigned char*)"ap2";
//			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[1],\
//					 OperateChange, 850-delta, 432, 970-delta, 432, uname, 1, 6, ColorGreen);
			
			//电容条
			uint8_t color;
			
			if(_superCapacity->cap_volt < 12)
				color = ColorOrange;
			else if(_superCapacity->cap_volt < 18)
				color = ColorYellow;
			else 
				color = ColorGreen;
			
			int Power_Line = (uint32_t)(((_superCapacity->cap_volt - 12) / 12.0f) * 600); //电容条
			
			uname = (unsigned char *)"cap";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[2],\
					 OperateChange, 700, 100, (700 + Power_Line), 100, uname, 40, 6, color);
			
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		case 7:
		{//吊射模式显示
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"dio";
				if(_robotState->follow==FOLLOW_ON)//吊射模式
				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
			else if(_robotState->follow==FOLLOW_OFF)//普通模式
				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off", 11, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
//			
			break;
		}
		case 8:
		{
			//图层七，摩擦轮开关标志
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char1;
			
			uname = (unsigned char*)"fri";
			if(_robotState->fric_mode == FRIC_ON)
				DrawChar(&char1, OperateChange, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: On  ", 10, ColorAmaranth);
			else
				DrawChar(&char1, OperateChange, 200, 680, uname, 2, 20, 7, (uint8_t*)"Fric: Close", 11, ColorGreen);
			
			CharacterToUIframe(&char1, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 9:
		{
			
			subcontent_id = 0x0103;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			
			InteractionFigure3Frame_t figure3= {};
			
			uint8_t color;
			if(_upperComputerComm->Receive.aiming_state == 0x33)
				color = ColorGreen;
			else
				color = ColorAmaranth;
			
			//检测自瞄通讯是否ok
			static uint32_t last_counter = 0, this_counter = 0;
			this_counter = _upperComputerComm->rec_counter;
			if(this_counter == last_counter)
				color = ColorBlack;
			last_counter = this_counter;

			uname = (unsigned char*)"aaa";
			DrawRect(&figure3.ext_interaction_figure_3.interaction_figure[0],\
			OperateChange, 695, 240, 1235, 710, uname, 5, 8, color);
			
			static uint8_t draw_flag = 0;
			if(_robotState->chassis_mode == CHASSIS_SEPARATE && !draw_flag)
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateAdd, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateAdd, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 1;
		    }
			else if( _robotState->chassis_mode != CHASSIS_SEPARATE && draw_flag )
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateDelete, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateDelete, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
				
				draw_flag = 0;
			}
			else if( _robotState->chassis_mode == CHASSIS_SEPARATE && draw_flag )
			{
				uname = (unsigned char*)"bbb";
				DrawCircle(&figure3.ext_interaction_figure_3.interaction_figure[1],\
				OperateChange, 1800, 700, 70, uname, 8, 8, ColorGreen);
				
				
				float begin = 0;
				float end = 0;
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 0)
				{
					begin = 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
					end = ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d <= 0) 
						        ? ( 390 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d )
								: ( 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
				}
				
				if(_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d < 0)
				{
					begin = ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d >= 360) 
						          ? ( -_chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d - 30 )
								  : ( 330 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d );
					end = 30 - _chassisControl->ChassisEstimate.gimbal_to_chassis_delta_angle_d;
				}
				
				uname = (unsigned char*)"ccc";
				DrawArc(&figure3.ext_interaction_figure_3.interaction_figure[2],\
				OperateChange, 1800, 700, 70, 70, begin, end, uname, 8, 8, ColorAmaranth);
			}
			
			uname = (unsigned char*)"fff";
			DrawLine(&figure3.ext_interaction_figure_3.interaction_figure[3],\
			         OperateChange, 910-delta, 540, 910-delta, 380, uname, 1, 8, ColorGreen);
			
			FigureToUIframe(figure3.data, subcontent_id, _UIframe->data);
			
			if(FigureJudge(figure3.ext_interaction_figure_3.interaction_figure,\
				subcontent_id) == LayerOk)
			{
				UIframeTransmit(_UIframe->data, subcontent_id);
				UIframeClear(_UIframe->data);
			}
			
			break;
		}
		
		case 10:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"tuy";
			if(_robotState->capacity_mode == NO_CAPACITY)
				DrawChar(&char3, OperateChange, 200, 730, uname, 2, 20, 6, (uint8_t*)"LOW POWER", 9, ColorAmaranth);
			else if(_robotState->capacity_mode == CAPACITY)
				DrawChar(&char3, OperateChange, 200, 730, uname, 2, 20, 6, (uint8_t*)"NORMAL POWER", 12, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
		case 11:
		{
			subcontent_id = 0x0110;
			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
			ext_interaction_character_t char3;
			
			uname = (unsigned char*)"fuk";
			if(_robotState->chassis_mode == CHASSIS_REVOLVE)
				DrawChar(&char3, OperateChange, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin On ", 8, ColorAmaranth);
			else
				DrawChar(&char3, OperateChange, 200, 780, uname, 2, 20, 6, (uint8_t*)"Spin Off ", 9, ColorGreen);
			
			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
			UIframeTransmit(_UIframe->data, subcontent_id);
			UIframeClear(_UIframe->data);
			
			break;
		}
		
//		case 11:
//			{//吊射模式显示
//			subcontent_id = 0x0110;
//			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
//			ext_interaction_character_t char3;
//			
//			uname = (unsigned char*)"dio";
//				if(_robotState->follow==FOLLOW_ON)//吊射模式
//				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow On ", 10, ColorAmaranth);
//			else if(_robotState->follow==FOLLOW_OFF)//普通模式
//				DrawChar(&char3, OperateChange, 200, 580, uname, 2, 20, 6, (uint8_t*)"Follow Off", 11, ColorGreen);
//			
//			CharacterToUIframe(&char3, subcontent_id, _UIframe->data);
//			UIframeTransmit(_UIframe->data, subcontent_id);
//			UIframeClear(_UIframe->data);
//			
//			break;
//		}
//		
//		case 12:
//		{
////			subcontent_id = 0x0110;
////			UIframeInit(_UIframe, subcontent_id, sender_id, receiver_id);
////			ext_interaction_character_t char2;
////			
////			uname = (unsigned char*)"dis";
////			DrawChar(&char2, OperateChange, 1080, 800, uname, 2, 20, 6, (uint8_t*)"DISTANCE:    	", 10, ColorYellow);
////			
////			CharacterToUIframe(&char2, subcontent_id, _UIframe->data);
////			UIframeTransmit(_UIframe->data, subcontent_id);
////			UIframeClear(_UIframe->data);
//		
//			break;
//		}
		
		default:
			break;
	}
	//需要画继续加，最多加到case 8
}
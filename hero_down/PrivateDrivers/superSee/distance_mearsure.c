#include "distance_measure.h"
LiDARFrameTypeDef distance_data;
float distance;
float upper_ddt;
void data_process(void) 
{
	static uint8_t cnt = 0;
	uint8_t i;
	static uint16_t count = 0;
	static float sum = 0;
	for(i=0;i<12;i++)
	{
		if(distance_data.point[i].distance != 0)
		{
			count++;
			sum += distance_data.point[i].distance;
		}
	}
	if(++cnt == 1)
	{
		static uint32_t cnnt;
		//upper_ddt = DWT_GetDeltaT(&cnnt);
		distance = sum/count;
		sum = 0;
		count = 0;
		cnt = 0;
	}
}
uint16_t receive_cnt;
void distance_datacheck(uint8_t data_buffer[]){
	static uint8_t crc = 0;
	static uint8_t cnt = 0;
	for(int index=0;index<47;index++){
	uint8_t temp_data = data_buffer[index];
			if (index > 5)
		{
			if(index < 42)
			{
				if(index%3 == 0)
				{
					distance_data.point[cnt].distance = (uint16_t)temp_data;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else if(index%3 == 1)
				{
					distance_data.point[cnt].distance = ((uint16_t)temp_data<<8)+distance_data.point[cnt].distance;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else
				{
					distance_data.point[cnt].intensity = temp_data;
					cnt++;	
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
			}
			else 
			{
				switch(index)
				{
					case 42:
						distance_data.end_angle = (uint16_t)temp_data;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 43:
						distance_data.end_angle = ((uint16_t)temp_data<<8)+distance_data.end_angle;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 44:
						distance_data.timestamp = (uint16_t)temp_data;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 45:
						distance_data.timestamp = ((uint16_t)temp_data<<8)+distance_data.timestamp;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 46:
						distance_data.crc8 = temp_data;
						if(distance_data.crc8 == crc)
						{
							data_process();
							receive_cnt++;
						}
						else
						{
						}
						crc = 0;
						cnt = 0;
					default: break;
				}
			}
		}
		else 
		{
			switch(index)
			{
				case 0:
					if(temp_data == HEADER)
					{
						distance_data.header = temp_data;
						crc = CrcTable[(crc^temp_data) & 0xff];
					} else crc = 0;
					break;
				case 1:
					//receive_cnt = temp_data;
					if(temp_data == VERLEN)
					{
						//receive_cnt++;
						distance_data.ver_len = temp_data;
						crc = CrcTable[(crc^temp_data) & 0xff];
					} else crc = 0;
					break;
				case 2:
					distance_data.temperature = (uint16_t)temp_data;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 3:
					distance_data.temperature = ((uint16_t)temp_data<<8)+distance_data.temperature;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 4:
					distance_data.start_angle = (uint16_t)temp_data;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 5:
					distance_data.start_angle = ((uint16_t)temp_data<<8)+distance_data.start_angle;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				default: break;
			}
		}
	}
}
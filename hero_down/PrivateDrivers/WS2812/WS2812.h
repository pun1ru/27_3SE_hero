/**
 * @file WS2812.h
 * @author 3SE
 * @brief WS2812 LED SPI通信
 * @version 
 * @date 2025-02-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"
#include "device_define.h"

#define WS2812_SPI_UNIT BOARD_LED_SPI
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
 
void WS2812_SPI_Ctrl(uint8_t r, uint8_t g, uint8_t b);
#endif

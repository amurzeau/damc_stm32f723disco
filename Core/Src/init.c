/*
 * init.c
 *
 *  Created on: Apr 30, 2023
 *      Author: doc
 */

#include "init.h"
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_ts.h>
#include <stm32f723e_discovery_lcd.h>
#include <stm32f723e_discovery_audio.h>
#include <math.h>

void TS_IO_Init();

void k_BspInit(void)
{
//  TS_IO_Init();
//
//  HAL_Delay(500);
//
//  BSP_TS_InitEx(240, 240, LCD_ORIENTATION_LANDSCAPE_ROT180);
//
//
//  uint8_t status = BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE_ROT180);
//
//  if (status != LCD_OK)
//  {
//    //ErrorHandler();
//  }
//
//  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
//  BSP_LCD_DisplayOn();
//  BSP_LCD_Clear(LCD_COLOR_BLACK);
//  BSP_LCD_DisplayStringAtLine(0, "Initializing...");
}

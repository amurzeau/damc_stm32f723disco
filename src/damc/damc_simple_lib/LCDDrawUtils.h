#pragma once

#include "fonts.h"
#include <stdint.h>

/**
 * @brief  LCD color in RGB565 format
 */
#define LCD_UTILS_COLOR_BLUE ((uint16_t) 0x001F)
#define LCD_UTILS_COLOR_GREEN ((uint16_t) 0x07E0)
#define LCD_UTILS_COLOR_RED ((uint16_t) 0xF800)
#define LCD_UTILS_COLOR_CYAN ((uint16_t) 0x07FF)
#define LCD_UTILS_COLOR_MAGENTA ((uint16_t) 0xF81F)
#define LCD_UTILS_COLOR_YELLOW ((uint16_t) 0xFFE0)
#define LCD_UTILS_COLOR_LIGHTBLUE ((uint16_t) 0x841F)
#define LCD_UTILS_COLOR_LIGHTGREEN ((uint16_t) 0x87F0)
#define LCD_UTILS_COLOR_LIGHTRED ((uint16_t) 0xFC10)
#define LCD_UTILS_COLOR_LIGHTMAGENTA ((uint16_t) 0xFC1F)
#define LCD_UTILS_COLOR_LIGHTYELLOW ((uint16_t) 0xFFF0)
#define LCD_UTILS_COLOR_DARKBLUE ((uint16_t) 0x0010)
#define LCD_UTILS_COLOR_DARKGREEN ((uint16_t) 0x0400)
#define LCD_UTILS_COLOR_DARKRED ((uint16_t) 0x8000)
#define LCD_UTILS_COLOR_DARKCYAN ((uint16_t) 0x0410)
#define LCD_UTILS_COLOR_DARKMAGENTA ((uint16_t) 0x8010)
#define LCD_UTILS_COLOR_DARKYELLOW ((uint16_t) 0x8400)
#define LCD_UTILS_COLOR_WHITE ((uint16_t) 0xFFFF)
#define LCD_UTILS_COLOR_LIGHTGRAY ((uint16_t) 0xD69A)
#define LCD_UTILS_COLOR_GRAY ((uint16_t) 0x8410)
#define LCD_UTILS_COLOR_DARKGRAY ((uint16_t) 0x4208)
#define LCD_UTILS_COLOR_BLACK ((uint16_t) 0x0000)
#define LCD_UTILS_COLOR_BROWN ((uint16_t) 0xA145)
#define LCD_UTILS_COLOR_ORANGE ((uint16_t) 0xFD20)

typedef struct {
	uint32_t TouchDetected; /* Touch detected : 0 means no, 1 means yes */
	uint32_t TouchX;        /* x coordinate */
	uint32_t TouchY;        /* y coordinate */
} TSState;

typedef struct {
	int16_t X;
	int16_t Y;
} LCDPoint;

/**
 * @brief  Line mode structures definition
 */
typedef enum {
	LCD_CENTER_MODE = 0x01, /* Center mode */
	LCD_RIGHT_MODE = 0x02,  /* Right mode  */
	LCD_LEFT_MODE = 0x03    /* Left mode   */
} LCDLineMode;

void LCDInit();
TSState TSGetState();
void LCDDrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code);
uint32_t LCDGetXSize();
uint32_t LCDGetYSize();
const sFONT* LCDGetFont();
void LCDDisplayOn();
void LCDDisplayOff();
void LCDFillScreen(uint32_t color);
void LCDFillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint32_t color);

void LCDDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t Color);
void LCDDrawPolygon(const LCDPoint* Points, uint16_t PointCount, uint32_t color);
void LCDDrawChar(uint16_t Xpos, uint16_t Ypos, uint32_t color, const uint8_t* c);
void LCDDisplayChar(uint16_t Xpos, uint16_t Ypos, uint32_t color, uint8_t Ascii);
void LCDDisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint32_t color, const char* Text, LCDLineMode Mode);

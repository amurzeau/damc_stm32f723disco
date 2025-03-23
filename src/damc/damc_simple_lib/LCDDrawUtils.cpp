#include "LCDDrawUtils.h"
#include "LCDController.h"
#include "fonts.h"

#ifdef STM32F723xx
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_lcd.h>
#include <stm32f723e_discovery_ts.h>
#include <stm32f7xx_hal_gpio.h>

void LCDInit() {
	// Initialize FRD154BP2902-D-CTQ LCD touch screen module

	// Initialize FT3267 touchscreen controller
	TS_IO_Init();
	HAL_Delay(500);
	BSP_TS_InitEx(240, 240, LCD_ORIENTATION_LANDSCAPE_ROT180);

	// Enable interrupt mode
	BSP_TS_ITConfig();

	// Initialize ST7789H2-G4 LCD controller
	uint8_t status = BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE_ROT180);
	if(status != LCD_OK) {
	}
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == TS_INT_PIN) {
		LCDController::instance->notifyTouchFromIrq();
	}
}

TSState TSGetState() {
	TS_StateTypeDef TS_State = {
	    .touchDetected = false,
	};

	BSP_TS_GetState(&TS_State);

	return TSState{
	    .TouchDetected = TS_State.touchDetected,
	    .TouchX = TS_State.touchX[0],
	    .TouchY = TS_State.touchY[0],
	};
}

uint32_t LCDGetXSize() {
	return BSP_LCD_GetXSize();
}

uint32_t LCDGetYSize() {
	return BSP_LCD_GetYSize();
}

const sFONT* LCDGetFont() {
	return &Font16;
}

void LCDDisplayOn() {
	BSP_LCD_DisplayOn();
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
}

void LCDDisplayOff() {
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_RESET);
	BSP_LCD_DisplayOff();
}

void LCDDrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code) {
	BSP_LCD_DrawPixel(Xpos, Ypos, RGB_Code);
}

void LCDFillScreen(uint32_t color) {
	BSP_LCD_SetTextColor(color);
	BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
}

void LCDFillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint32_t color) {
	BSP_LCD_SetTextColor(color);
	BSP_LCD_FillRect(Xpos, Ypos, Xsize, Ysize);
}

#elif defined(STM32N657xx)
#include <stm32n6570_discovery.h>
#include <stm32n6570_discovery_lcd.h>
#include <stm32n6570_discovery_ts.h>
#include <stm32n6xx_hal_gpio.h>

void LCDInit() {
	// Initialize FRD154BP2902-D-CTQ LCD touch screen module

	// Initialize FT3267 touchscreen controller
	TS_Init_t tsInit = {
	    .Width = LCD_DEFAULT_WIDTH,
	    .Height = LCD_DEFAULT_HEIGHT,
	    .Orientation = LCD_ORIENTATION_LANDSCAPE,
	    .Accuracy = 10,
	};
	BSP_TS_Init(0, &tsInit);

	// Enable interrupt mode
	BSP_TS_EnableIT(0);

	// Initialize ST7789H2-G4 LCD controller
	BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
}

extern "C" void BSP_TS_Callback(uint32_t Instance) {
	LCDController::instance->notifyTouchFromIrq();
}

TSState TSGetState() {
	TS_State_t state;
	BSP_TS_GetState(0, &state);

	return TSState{
	    .TouchDetected = state.TouchDetected,
	    .TouchX = state.TouchX,
	    .TouchY = state.TouchY,
	};
}

void LCDDrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code) {
	BSP_LCD_WritePixel(0, Xpos, Ypos, RGB_Code);
}

uint32_t LCDGetXSize() {
	return LCD_DEFAULT_WIDTH;
}

uint32_t LCDGetYSize() {
	return LCD_DEFAULT_HEIGHT;
}

const sFONT* LCDGetFont() {
	return &Font16;
}

void LCDDisplayOn() {
	BSP_LCD_DisplayOn(0);
	HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_6, GPIO_PIN_SET); /* 100% Brightness */ /* PQ6  LCD_BL_CTRL */
}

void LCDDisplayOff() {
	HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_6, GPIO_PIN_RESET);
	BSP_LCD_DisplayOff(0);
}

void LCDFillScreen(uint32_t color) {
	BSP_LCD_FillRect(0, 0, 0, LCD_DEFAULT_WIDTH, LCD_DEFAULT_HEIGHT, color);
}

void LCDFillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint32_t color) {
	BSP_LCD_FillRect(0, Xpos, Ypos, Xsize, Ysize, color);
}

#endif

#ifndef ABS
#define ABS(X) ((X) > 0 ? (X) : -(X))
#endif

/**
 * @brief  Draws an uni-line (between two points).
 * @param  x1: Point 1 X position
 * @param  y1: Point 1 Y position
 * @param  x2: Point 2 X position
 * @param  y2: Point 2 Y position
 * @retval None
 */
void LCDDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t Color) {
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, yinc1 = 0, yinc2 = 0, den = 0, num = 0,
	        numadd = 0, numpixels = 0, curpixel = 0;

	deltax = ABS(x2 - x1); /* The difference between the x's */
	deltay = ABS(y2 - y1); /* The difference between the y's */
	x = x1;                /* Start x off at the first pixel */
	y = y1;                /* Start y off at the first pixel */

	if(x2 >= x1) /* The x-values are increasing */
	{
		xinc1 = 1;
		xinc2 = 1;
	} else /* The x-values are decreasing */
	{
		xinc1 = -1;
		xinc2 = -1;
	}

	if(y2 >= y1) /* The y-values are increasing */
	{
		yinc1 = 1;
		yinc2 = 1;
	} else /* The y-values are decreasing */
	{
		yinc1 = -1;
		yinc2 = -1;
	}

	if(deltax >= deltay) /* There is at least one x-value for every y-value */
	{
		xinc1 = 0; /* Don't change the x when numerator >= denominator */
		yinc2 = 0; /* Don't change the y for every iteration */
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax; /* There are more x-values than y-values */
	} else                  /* There is at least one y-value for every x-value */
	{
		xinc2 = 0; /* Don't change the x for every iteration */
		yinc1 = 0; /* Don't change the y when numerator >= denominator */
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay; /* There are more y-values than x-values */
	}

	for(curpixel = 0; curpixel <= numpixels; curpixel++) {
		LCDDrawPixel(x, y, Color); /* Draw the current pixel */
		num += numadd;             /* Increase the numerator by the top of the fraction */
		if(num >= den)             /* Check if numerator >= denominator */
		{
			num -= den; /* Calculate the new numerator value */
			x += xinc1; /* Change the x as appropriate */
			y += yinc1; /* Change the y as appropriate */
		}
		x += xinc2; /* Change the x as appropriate */
		y += yinc2; /* Change the y as appropriate */
	}
}

/**
 * @brief  Draws an poly-line (between many points).
 * @param  Points: Pointer to the points array
 * @param  PointCount: Number of points
 * @retval None
 */
void LCDDrawPolygon(const LCDPoint* Points, uint16_t PointCount, uint32_t color) {
	int16_t x = 0, y = 0;

	if(PointCount < 2) {
		return;
	}

	LCDDrawLine(Points->X, Points->Y, (Points + PointCount - 1)->X, (Points + PointCount - 1)->Y, color);

	while(--PointCount) {
		x = Points->X;
		y = Points->Y;
		Points++;
		LCDDrawLine(x, y, Points->X, Points->Y, color);
	}
}

/**
 * @brief  Draws a character on LCD.
 * @param  Xpos: Line where to display the character shape
 * @param  Ypos: Start column address
 * @param  c: Pointer to the character data
 * @retval None
 */
void LCDDrawChar(uint16_t Xpos, uint16_t Ypos, uint32_t color, const uint8_t* c) {
	uint32_t i = 0, j = 0;
	uint16_t height, width;
	uint8_t offset;
	uint8_t* pchar;
	uint32_t line;

	height = LCDGetFont()->Height;
	width = LCDGetFont()->Width;

	offset = 8 * ((width + 7) / 8) - width;

	for(i = 0; i < height; i++) {
		pchar = ((uint8_t*) c + (width + 7) / 8 * i);

		switch(((width + 7) / 8)) {
			case 1:
				line = pchar[0];
				break;

			case 2:
				line = (pchar[0] << 8) | pchar[1];
				break;

			case 3:
			default:
				line = (pchar[0] << 16) | (pchar[1] << 8) | pchar[2];
				break;
		}

		for(j = 0; j < width; j++) {
			if(line & (1 << (width - j + offset - 1))) {
				LCDDrawPixel((Xpos + j), Ypos, color);
			}
		}
		Ypos++;
	}
}

/**
 * @brief  Displays one character.
 * @param  Xpos: Start column address
 * @param  Ypos: Line where to display the character shape.
 * @param  Ascii: Character ascii code
 *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E
 * @retval None
 */
void LCDDisplayChar(uint16_t Xpos, uint16_t Ypos, uint32_t color, uint8_t Ascii) {
	LCDDrawChar(Xpos,
	            Ypos,
	            color,
	            &LCDGetFont()->table[(Ascii - ' ') * LCDGetFont()->Height * ((LCDGetFont()->Width + 7) / 8)]);
}

/**
 * @brief  Displays characters on the LCD.
 * @param  Xpos: X position (in pixel)
 * @param  Ypos: Y position (in pixel)
 * @param  Text: Pointer to string to display on LCD
 * @param  Mode: Display mode
 *          This parameter can be one of the following values:
 *            @arg  CENTER_MODE
 *            @arg  RIGHT_MODE
 *            @arg  LEFT_MODE
 * @retval None
 */
void LCDDisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint32_t color, const char* Text, LCDLineMode Mode) {
	int16_t refcolumn, refline;
	uint16_t i = 0;
	uint32_t size = 0;
	const char* ptr = Text;

	/* Get the text size */
	while(*ptr++)
		size++;

	switch(Mode) {
		case LCD_CENTER_MODE: {
			refcolumn = Xpos - (size * LCDGetFont()->Width) / 2;
			refline = Ypos - LCDGetFont()->Height / 2;
			break;
		}
		case LCD_LEFT_MODE: {
			refcolumn = Xpos;
			refline = Ypos;
			break;
		}
		case LCD_RIGHT_MODE: {
			refcolumn = Xpos - (size * LCDGetFont()->Width);
			refline = Ypos;
			break;
		}
		default: {
			refcolumn = Xpos;
			refline = Ypos;
			break;
		}
	}

	/* Check that the Start column is located in the screen */
	if(refcolumn > (int) LCDGetXSize()) {
		refcolumn = 0;
	}

	if(refline > (int) LCDGetYSize()) {
		refline = 0;
	}

	/* Send the string character by character on lCD */
	while((*Text != 0) & (((LCDGetXSize() - (i * LCDGetFont()->Width)) & 0xFFFF) >= LCDGetFont()->Width)) {
		/* Display one character on LCD */
		LCDDisplayChar(refcolumn, refline, color, *Text);
		/* Decrement the column position by 16 */
		refcolumn += LCDGetFont()->Width;
		/* Point on the next character */
		Text++;
		i++;
	}
}

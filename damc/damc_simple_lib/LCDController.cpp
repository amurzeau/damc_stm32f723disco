#include "LCDController.h"
#include <Osc/OscReadOnlyVariable.h>
#include <OscRoot.h>
#include <Utils.h>
#include <assert.h>
#include <math.h>
#include <spdlog/spdlog.h>
#include <string.h>

#include "../Components/Fonts/fonts.h"
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_lcd.h>
#include <stm32f723e_discovery_ts.h>

LCDController* LCDController::instance;

struct OscPanelLinkDeclaration {
	const char* nodes[4];  // at most 4 nodes are configurable (4x one button for bool values)
};

LCDController::LCDController(OscRoot* oscRoot)
    : oscRoot(oscRoot), touchX(0), touchY(0), menuPath{"DAMC"}, menusState{} {
	instance = this;
	oscStrips = oscRoot->getNode("strip");
	touchHandlers.reserve(5);
}

void LCDController::start() {
	// Initialize FRD154BP2902-D-CTQ LCD touch screen module

	// Initialize FT3267 touchscreen controller
	TS_IO_Init();
	HAL_Delay(500);
	BSP_TS_InitEx(240, 240, LCD_ORIENTATION_LANDSCAPE_ROT180);

	// Initialize ST7789H2-G4 LCD controller
	uint8_t status = BSP_LCD_InitEx(LCD_ORIENTATION_LANDSCAPE_ROT180);
	if(status != LCD_OK) {
	}
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	// BSP_LCD_DisplayStringAtLine(0, "Initializing...");
	BSP_LCD_DisplayOn();

	drawMenuMain();
}

void LCDController::mainLoop() {
	TS_StateTypeDef TS_State;
	BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected > 0 && !touchIsPressed) {
		touchIsPressed = true;
		handleClick(240 - TS_State.touchX[0], TS_State.touchY[0]);
	} else if(TS_State.touchDetected == 0 && touchIsPressed) {
		touchIsPressed = false;
	}
}

void LCDController::drawIcon(Icon icon, int x, int y, int width, int height) {
	switch(icon) {
		case ICON_CONFIGURE: {
			int line1Y = y + 3;
			int line2Y = y + height - 3;
			BSP_LCD_DrawLine(x, line1Y, x + width, line1Y);
			BSP_LCD_DrawLine(x, line2Y, x + width, line2Y);
			BSP_LCD_DrawCircle(x + 3 * width / 4, line1Y, 3);
			BSP_LCD_DrawCircle(x + width / 4, line2Y, 3);
			break;
		}
		case ICON_ARROW_UP: {
			int pointsX[3] = {x + width / 2, x, x + width};
			int pointsY[3] = {y, y + height, y + height};

			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1]);
			BSP_LCD_DrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2]);
			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2]);
			break;
		}
		case ICON_ARROW_DOWN: {
			int pointsX[3] = {x + width / 2, x, x + width};
			int pointsY[3] = {y + height, y, y};

			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1]);
			BSP_LCD_DrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2]);
			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2]);
			break;
		}
		case ICON_ARROW_LEFT: {
			int pointsX[3] = {x, x + width, x + width};
			int pointsY[3] = {y + height / 2, y, y + height};

			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1]);
			BSP_LCD_DrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2]);
			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2]);
			break;
		}
		case ICON_ARROW_RIGHT: {
			int pointsX[3] = {x + width, x, x};
			int pointsY[3] = {y + height / 2, y, y + height};

			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1]);
			BSP_LCD_DrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2]);
			BSP_LCD_DrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2]);
			break;
		}
		case ICON_ARROW_BACK: {
			int16_t xMiddle = x + width * 8 / 24;
			int16_t yMiddle = y + height / 2;
			int16_t yMidPoint1 = y + height / 3;
			int16_t yMidPoint2 = y + 2 * height / 3;
			Point arrowLeftPoints[] = {
			    {(int16_t) (x), (int16_t) (yMiddle)},
			    {(int16_t) (xMiddle), (int16_t) (y)},
			    {(int16_t) (xMiddle), (int16_t) (yMidPoint1)},
			    {(int16_t) (x + width), (int16_t) (yMidPoint1)},
			    {(int16_t) (x + width), (int16_t) (yMidPoint2)},
			    {(int16_t) (xMiddle), (int16_t) (yMidPoint2)},
			    {(int16_t) (xMiddle), (int16_t) (y + height)},
			    {(int16_t) (x), (int16_t) (yMiddle)},
			};
			BSP_LCD_DrawPolygon(arrowLeftPoints, sizeof(arrowLeftPoints) / sizeof(arrowLeftPoints[0]));
			break;
		}
		case ICON_BUTTON_RELEASED: {
			Point arrowLeftPoints[] = {
			    {(int16_t) (x), (int16_t) (y)},
			    {(int16_t) (x), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y)},
			};
			BSP_LCD_DrawPolygon(arrowLeftPoints, sizeof(arrowLeftPoints) / sizeof(arrowLeftPoints[0]));
			break;
		}
		case ICON_BUTTON_PRESSED:
			Point arrowLeftPoints[] = {
			    {(int16_t) (x), (int16_t) (y)},
			    {(int16_t) (x), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y)},
			};
			BSP_LCD_FillRect(x, y, width, height);
			BSP_LCD_DrawPolygon(arrowLeftPoints, sizeof(arrowLeftPoints) / sizeof(arrowLeftPoints[0]));
			break;
	}
}

void LCDController::clearScreen() {
	BSP_LCD_Clear(LCD_COLOR_BLACK);
}

void LCDController::drawHeader() {
	size_t numberOfColumnsDrawn = 0;
	size_t charWidth = BSP_LCD_GetFont()->Width;

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

	for(size_t i = 0; menuPath[i] && numberOfColumnsDrawn < 240; i++) {
		const char* menu = menuPath[i];

		if(i > 0) {
			// Print separator
			BSP_LCD_DisplayStringAt(numberOfColumnsDrawn, 0, ">", LEFT_MODE);
			numberOfColumnsDrawn += 1 * charWidth;
		}

		BSP_LCD_DisplayStringAt(numberOfColumnsDrawn, 0, menu, LEFT_MODE);
		numberOfColumnsDrawn += strlen(menu) * charWidth;
	}

	if(menuPath[1]) {
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		drawIcon(ICON_ARROW_BACK, 2, 8, 24, 16);
	}
}

void LCDController::drawPanelButtonX1(PanelPosition panelIndex, const char* buttonName, bool buttonState) {
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	int16_t offset = panelIndex == PP_Left ? 0 : 120;
	drawIcon(buttonState ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED, 6 + offset, 114, 108, 60);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 114 + 60 / 2, buttonName, CENTER_MODE);
}

void LCDController::drawPanelButtonX2(
    PanelPosition panelIndex, const char* button1Name, bool button1State, const char* button2Name, bool button2State) {
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	int16_t offset = panelIndex == PP_Left ? 0 : 120;

	drawIcon(button1State ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED, 6 + offset, 60, 108, 60);
	drawIcon(button2State ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED, 6 + offset, 168, 108, 60);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 60 + 60 / 2, button1Name, CENTER_MODE);
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 168 + 60 / 2, button2Name, CENTER_MODE);
}

void LCDController::drawPanelConfigEnum(PanelPosition panelIndex,
                                        const char* title,
                                        const char* enumString,
                                        bool arrowUpAvailable,
                                        bool arrowDownAvailable) {
	int16_t offset = panelIndex == PP_Left ? 0 : 120;

	// Groupbox
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	Point groupBoxPoints[] = {
	    {(int16_t) (6 + offset), (int16_t) (24)},
	    {(int16_t) (6 + offset), (int16_t) (24 + 210)},
	    {(int16_t) (114 + offset), (int16_t) (24 + 210)},
	    {(int16_t) (114 + offset), (int16_t) (24)},
	};
	BSP_LCD_DrawPolygon(groupBoxPoints, sizeof(groupBoxPoints) / sizeof(groupBoxPoints[0]));

	// Up arrow
	if(arrowUpAvailable) {
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	drawIcon(ICON_ARROW_UP, 6 + offset, 60, 108, 60);

	// Down arrow
	if(arrowDownAvailable) {
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	drawIcon(ICON_ARROW_DOWN, 6 + offset, 168, 108, 60);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	// Title string
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 24 + 30 / 2, title, CENTER_MODE);

	// Value string
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 129 + 30 / 2, enumString, CENTER_MODE);
}

void LCDController::drawPanelConfigFloat(PanelPosition panelIndex,
                                         const char* title,
                                         float value,
                                         const char* valueSuffix) {
	int16_t offset = panelIndex == PP_Left ? 0 : 120;

	// Groupbox
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	Point groupBoxPoints[] = {
	    {(int16_t) (6 + offset), (int16_t) (24)},
	    {(int16_t) (6 + offset), (int16_t) (24 + 210)},
	    {(int16_t) (114 + offset), (int16_t) (24 + 210)},
	    {(int16_t) (114 + offset), (int16_t) (24)},
	};
	BSP_LCD_DrawPolygon(groupBoxPoints, sizeof(groupBoxPoints) / sizeof(groupBoxPoints[0]));

	// Up arrow
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	drawIcon(ICON_ARROW_UP, 6 + offset, 60, 108, 60);

	// Down arrow
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	drawIcon(ICON_ARROW_DOWN, 6 + offset, 168, 108, 60);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	// Title string
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 24 + 30 / 2, title, CENTER_MODE);

	// Value string
	int integerPart = (int) value;
	int decimalPart = abs((int) (value * 100)) % 100;
	char buffer[8];

	if(decimalPart) {
		snprintf(buffer, sizeof(buffer), "%d.%02d%s", integerPart, decimalPart, valueSuffix);
	} else {
		snprintf(buffer, sizeof(buffer), "%d%s", integerPart, valueSuffix);
	}
	buffer[7] = '\0';
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 129 + 30 / 2, buffer, CENTER_MODE);
}

const char* LCDController::getStripDisplayStringNode(size_t stripIndex) {
	if(!oscStrips)
		return "";

	std::string_view indexStr = Utils::toString(stripIndex);
	OscNode* stripNode = oscStrips->getNode(indexStr);
	if(!stripNode)
		return "";

	menusState.muteNode = (OscReadOnlyVariable<bool>*) stripNode->getNode("filterChain/mute");
	menusState.volumeNode = (OscReadOnlyVariable<float>*) stripNode->getNode("filterChain/volume");

	OscReadOnlyVariable<std::string>* displayStringNode =
	    (OscReadOnlyVariable<std::string>*) stripNode->getNode("display_name");
	if(!displayStringNode)
		return "";

	return displayStringNode->get().c_str();
}

void LCDController::pushMenu(const char* previousMenuName) {
	size_t i;
	for(i = 0; i + 1 < sizeof(menuPath) / sizeof(menuPath[0]) && menuPath[i] != nullptr; i++)
		;

	if(i + 1 < sizeof(menuPath) / sizeof(menuPath[0])) {
		menuPath[i] = previousMenuName;
		menuPath[i + 1] = nullptr;
	}
}
void LCDController::popMenu() {
	size_t i;
	for(i = 0; i + 1 < sizeof(menuPath) / sizeof(menuPath[0]) && menuPath[i] != nullptr; i++)
		;

	if(i > 0) {
		menuPath[i - 1] = nullptr;
	}
}

void LCDController::drawMenuMain() {
	const char* currentStripName = getStripDisplayStringNode(menusState.stripIndex);
	clearScreen();
	drawHeader();

	drawPanelConfigEnum(PP_Left, "Strip", currentStripName, menusState.stripIndex < 4, menusState.stripIndex > 0);
	drawPanelButtonX1(PP_Right, "Config", false);
	touchHandlers.clear();
	touchHandlers.push_back(TouchHandler{6, 60, 108, 60, [](LCDController* thisInstance) {
		                                     if(thisInstance->menusState.stripIndex < 4) {
			                                     thisInstance->menusState.stripIndex++;
			                                     thisInstance->drawMenuMain();
		                                     }
	                                     }});
	touchHandlers.push_back(TouchHandler{6, 168, 108, 60, [](LCDController* thisInstance) {
		                                     if(thisInstance->menusState.stripIndex > 0) {
			                                     thisInstance->menusState.stripIndex--;
			                                     thisInstance->drawMenuMain();
		                                     }
	                                     }});
	touchHandlers.push_back(TouchHandler{
	    6 + 120, 114, 108, 60, [](LCDController* thisInstance) {
		    thisInstance->pushMenu(thisInstance->getStripDisplayStringNode(thisInstance->menusState.stripIndex));
		    thisInstance->drawMenuStripConfig();
	    }});
}

void LCDController::drawMenuStripConfig() {
	clearScreen();
	drawHeader();

	drawPanelButtonX2(PP_Left, "Mute", menusState.muteNode->get(), "Config", false);
	drawPanelConfigFloat(PP_Right, "Volume", menusState.volumeNode->getToOsc(), "dB");
	touchHandlers.clear();
	touchHandlers.push_back(TouchHandler{6, 60, 108, 60, [](LCDController* thisInstance) {
		                                     thisInstance->menusState.muteNode->set(
		                                         !thisInstance->menusState.muteNode->get());
		                                     thisInstance->drawMenuStripConfig();
	                                     }});
	touchHandlers.push_back(TouchHandler{6 + 120, 60, 108, 60, [](LCDController* thisInstance) {
		                                     thisInstance->menusState.volumeNode->setFromOsc(
		                                         thisInstance->menusState.volumeNode->getToOsc() + 1);
		                                     thisInstance->drawMenuStripConfig();
	                                     }});
	touchHandlers.push_back(TouchHandler{6 + 120, 168, 108, 60, [](LCDController* thisInstance) {
		                                     thisInstance->menusState.volumeNode->setFromOsc(
		                                         thisInstance->menusState.volumeNode->getToOsc() - 1);
		                                     thisInstance->drawMenuStripConfig();
	                                     }});
	touchHandlers.push_back(TouchHandler{0, 0, 120, 60, [](LCDController* thisInstance) {
		                                     thisInstance->popMenu();
		                                     thisInstance->drawMenuMain();
	                                     }});
}

void LCDController::handleClick(int x, int y) {
	BSP_LCD_DrawPixel(touchX, touchY, 0x0000);
	touchX = x;
	touchY = y;
	BSP_LCD_DrawPixel(touchX, touchY, 0xFFFF);

	for(const auto& touchHandler : touchHandlers) {
		if((x >= touchHandler.x && x <= touchHandler.x + touchHandler.width) &&
		   (y >= touchHandler.y && y <= touchHandler.y + touchHandler.height))
			touchHandler.handler(this);
	}
}

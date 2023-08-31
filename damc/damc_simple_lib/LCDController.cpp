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

struct ConfigMapping {
	const char* oscName;
	const char* displayName;
};

static constexpr ConfigMapping COMPRESSOR_CONFIG_NAMES[] = {
    {"makeUpgain", "St. Gain"},
    {"ratio", "Ratio"},
    {"kneeWidth", "K. Width"},
    {"threshold", "Thres"},
    {"releaseTime", "Release"},
    {"attackTime", "Attack"},
    {"lufsTarget", "L Target"},
    {"lufsIntegTime", "L Time"},
    {"lufsGate", "L Gate"},
};

static constexpr ConfigMapping EXPANDER_CONFIG_NAMES[] = {
    {"makeUpgain", "St. Gain"},
    {"ratio", "Ratio"},
    {"kneeWidth", "K. Width"},
    {"threshold", "Thres"},
    {"releaseTime", "Release"},
    {"attackTime", "Attack"},
};

struct OscPanelLinkDeclaration {
	const char* nodes[4];  // at most 4 nodes are configurable (4x one button for bool values)
};

LCDController::LCDController(OscRoot* oscRoot)
    : oscRoot(oscRoot),
      touchX(0),
      touchY(0),
      touchIsPressed(false),
      lcdIsOn(false),
      touchLastPressTime(0),
      menuHistorySize(0),
      menusState{} {
	instance = this;
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

	pushMenu("DAMC", &LCDController::drawMenuMain);
	drawCurrentMenu();

	touchLastPressTime = HAL_GetTick();
	lcdTurnOn();
}

void LCDController::mainLoop() {
	TS_StateTypeDef TS_State;

	uint32_t currentTick = HAL_GetTick();

	BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected > 0 && !touchIsPressed) {
		touchIsPressed = true;
		touchLastPressTime = currentTick;
		if(!lcdIsOn) {
			lcdTurnOn();
		} else {
			handleClick(240 - TS_State.touchX[0], TS_State.touchY[0]);
		}
	} else if(TS_State.touchDetected == 0 && touchIsPressed) {
		touchIsPressed = false;
	} else if(!touchIsPressed && lcdIsOn && touchLastPressTime + 30000 < currentTick) {
		// Touchscreen not pressed and LCD is ON
		// 30s elapsed => turn off LCD screen and backlight
		lcdTurnOff();
	}
}

void LCDController::lcdTurnOn() {
	lcdIsOn = true;
	BSP_LCD_DisplayOn();
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
}
void LCDController::lcdTurnOff() {
	lcdIsOn = false;
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_RESET);
	BSP_LCD_DisplayOff();
}

void LCDController::drawIcon(Icon icon, int x, int y, int width, int height, TouchHandleCallback onClick) {
	switch(icon) {
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
	if(onClick)
		touchHandlers.push_back({x, y, width, height, onClick});
}

void LCDController::clearScreen() {
	BSP_LCD_Clear(LCD_COLOR_BLACK);
}

void LCDController::drawHeader() {
	size_t numberOfColumnsDrawn = 0;
	size_t charWidth = BSP_LCD_GetFont()->Width;

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

	for(size_t i = 0; i < menuHistorySize && numberOfColumnsDrawn < 240; i++) {
		MenuInfo* menu = &menuHistory[i];

		if(i > 0) {
			// Print separator
			BSP_LCD_DisplayStringAt(numberOfColumnsDrawn, 0, ">", LEFT_MODE);
			numberOfColumnsDrawn += 1 * charWidth;
		}

		BSP_LCD_DisplayStringAt(numberOfColumnsDrawn, 0, menu->name, LEFT_MODE);
		numberOfColumnsDrawn += strlen(menu->name) * charWidth;
	}

	if(menuHistorySize > 1) {
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		drawIcon(ICON_ARROW_BACK, 2, 8, 24, 16, nullptr);
		// Use hardcoded zone for a easier touch
		touchHandlers.push_back({0, 0, 120, 60, [](LCDController* thisInstance) { thisInstance->popMenu(); }});
	}
}

void LCDController::drawPanelButtonX1(PanelPosition panelIndex,
                                      const char* buttonName,
                                      bool buttonState,
                                      TouchHandleCallback buttonClick) {
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	int16_t offset = panelIndex == PP_Left ? 0 : 120;
	drawIcon(buttonState ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED, 6 + offset, 114, 108, 60, buttonClick);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 114 + 60 / 2, buttonName, CENTER_MODE);
}

void LCDController::drawPanelButtonX2(PanelPosition panelIndex,
                                      const char* button1Name,
                                      bool button1State,
                                      TouchHandleCallback button1Click,
                                      const char* button2Name,
                                      bool button2State,
                                      TouchHandleCallback button2Click) {
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	int16_t offset = panelIndex == PP_Left ? 0 : 120;

	drawIcon(button1State ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED, 6 + offset, 60, 108, 60, button1Click);
	drawIcon(button2State ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED, 6 + offset, 168, 108, 60, button2Click);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 60 + 60 / 2, button1Name, CENTER_MODE);
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 168 + 60 / 2, button2Name, CENTER_MODE);
}

void LCDController::drawPanelConfigEnum(PanelPosition panelIndex,
                                        const char* title,
                                        const char* enumString,
                                        bool arrowUpAvailable,
                                        bool arrowDownAvailable,
                                        TouchHandleCallback upClick,
                                        TouchHandleCallback downClick) {
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
	drawIcon(ICON_ARROW_UP, 6 + offset, 60, 108, 60, upClick);

	// Down arrow
	if(arrowDownAvailable) {
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
	drawIcon(ICON_ARROW_DOWN, 6 + offset, 168, 108, 60, downClick);

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	// Title string
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 24 + 30 / 2, title, CENTER_MODE);

	// Value string
	BSP_LCD_DisplayStringAt(6 + 108 / 2 + offset, 129 + 30 / 2, enumString, CENTER_MODE);
}

void LCDController::drawPanelConfigFloat(PanelPosition panelIndex,
                                         const char* title,
                                         float value,
                                         const char* valueSuffix,
                                         TouchHandleCallback upClick,
                                         TouchHandleCallback downClick) {
	int integerPart = (int) value;
	int decimalPart = abs((int) (value * 100)) % 100;
	char buffer[8];

	if(decimalPart) {
		snprintf(buffer, sizeof(buffer), "%d.%02d%s", integerPart, decimalPart, valueSuffix);
	} else {
		snprintf(buffer, sizeof(buffer), "%d%s", integerPart, valueSuffix);
	}
	buffer[7] = '\0';
	drawPanelConfigEnum(panelIndex, title, buffer, true, true, upClick, downClick);
}

void LCDController::pushMenu(const char* name, MenuHandlerCallback menuHandler) {
	if(menuHistorySize >= sizeof(menuHistory) / sizeof(menuHistory[0]))
		return;

	menuHistory[menuHistorySize].name = name;
	menuHistory[menuHistorySize].menuHandler = menuHandler;
	menuHistorySize++;
}
void LCDController::popMenu() {
	if(menuHistorySize <= 1)
		return;

	menuHistorySize--;
}

void LCDController::drawCurrentMenu() {
	clearScreen();
	touchHandlers.clear();

	drawHeader();
	if(menuHistorySize > 0) {
		(this->*menuHistory[menuHistorySize - 1].menuHandler)();
	}
}

void LCDController::updateStripDisplayString() {
	size_t stripIndex = menusState.stripIndex;

	menusState.stripDisplayName = "";

	OscNode* oscStrips = oscRoot->getNode("strip");
	if(!oscStrips)
		return;

	std::string_view indexStr = Utils::toString(stripIndex);
	OscNode* stripNode = oscStrips->getNode(indexStr);
	if(!stripNode)
		return;

	menusState.muteNode = (OscReadOnlyVariable<bool>*) stripNode->getNode("filterChain/mute");
	menusState.volumeNode = (OscReadOnlyVariable<float>*) stripNode->getNode("filterChain/volume");

	OscReadOnlyVariable<std::string>* displayStringNode =
	    (OscReadOnlyVariable<std::string>*) stripNode->getNode("display_name");
	if(!displayStringNode)
		return;

	menusState.stripDisplayName = displayStringNode->get().c_str();
}

void LCDController::drawMenuMain() {
	if(!menusState.stripDisplayName) {
		updateStripDisplayString();
	}

	drawPanelConfigEnum(
	    PP_Left,
	    "Strip",
	    menusState.stripDisplayName,
	    menusState.stripIndex < 4,
	    menusState.stripIndex > 0,
	    [](LCDController* thisInstance) {
		    if(thisInstance->menusState.stripIndex < 4) {
			    thisInstance->menusState.stripIndex++;
			    thisInstance->updateStripDisplayString();
		    }
	    },
	    [](LCDController* thisInstance) {
		    if(thisInstance->menusState.stripIndex > 0) {
			    thisInstance->menusState.stripIndex--;
			    thisInstance->updateStripDisplayString();
		    }
	    });
	drawPanelButtonX1(PP_Right, "Config", false, [](LCDController* thisInstance) {
		thisInstance->pushMenu(thisInstance->menusState.stripDisplayName, &LCDController::drawMenuStripConfig);
	});
}

void LCDController::drawMenuStripConfig() {
	drawPanelButtonX2(
	    PP_Left,
	    "Mute",
	    menusState.muteNode->get(),
	    [](LCDController* thisInstance) {
		    thisInstance->menusState.muteNode->set(!thisInstance->menusState.muteNode->get());
		    thisInstance->drawMenuStripConfig();
	    },
	    "Config",
	    false,
	    nullptr);
	drawPanelConfigFloat(
	    PP_Right,
	    "Volume",
	    menusState.volumeNode->getToOsc(),
	    "dB",
	    [](LCDController* thisInstance) {
		    thisInstance->menusState.volumeNode->setFromOsc(thisInstance->menusState.volumeNode->getToOsc() + 1);
		    thisInstance->drawMenuStripConfig();
	    },

	    [](LCDController* thisInstance) {
		    thisInstance->menusState.volumeNode->setFromOsc(thisInstance->menusState.volumeNode->getToOsc() - 1);
		    thisInstance->drawMenuStripConfig();
	    });
}

void LCDController::handleClick(int x, int y) {
	BSP_LCD_DrawPixel(touchX, touchY, 0x0000);
	touchX = x;
	touchY = y;
	BSP_LCD_DrawPixel(touchX, touchY, 0xFFFF);

	for(const auto& touchHandler : touchHandlers) {
		if((x >= touchHandler.x && x <= touchHandler.x + touchHandler.width) &&
		   (y >= touchHandler.y && y <= touchHandler.y + touchHandler.height)) {
			touchHandler.handler(this);
			drawCurrentMenu();
		}
	}
}

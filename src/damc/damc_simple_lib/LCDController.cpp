#include "LCDController.h"
#include "LCDDrawUtils.h"
#include <Osc/OscReadOnlyVariable.h>
#include <OscRoot.h>
#include <Utils.h>
#include <assert.h>
#include <math.h>
#include <spdlog/spdlog.h>
#include <string.h>

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
    : oscRoot(oscRoot), touchX(0), touchY(0), touchIsPressed(true), lcdIsOn(false), menuHistorySize(0), menusState{} {
	instance = this;
	touchHandlers.reserve(5);
	uv_timer_init(uv_default_loop(), &timerLcdOff);
	timerLcdOff.data = this;
	uv_async_init(uv_default_loop(), &asyncNotifyTouchIrq, LCDController::onTouchIrqStatic);
	asyncNotifyTouchIrq.data = this;
}

void LCDController::start() {
	LCDInit();

	LCDFillScreen(LCD_UTILS_COLOR_BLACK);

	pushMenu("DAMC", &LCDController::drawMenuMain);
	drawCurrentMenu();

	lcdTurnOn();
}

void LCDController::notifyTouchFromIrq() {
	uv_async_send(&asyncNotifyTouchIrq);
}

void LCDController::onTouchIrqStatic(uv_async_t* handle) {
	LCDController* thisInstance = (LCDController*) handle->data;
	thisInstance->onTouchIrq();
}

void LCDController::onTouchIrq() {
	TSState state = TSGetState();

	if(state.TouchDetected > 0 && !touchIsPressed) {
		touchIsPressed = true;

		uv_timer_stop(&timerLcdOff);

		if(!lcdIsOn) {
			lcdTurnOn();
		} else {
			handleClick(240 - state.TouchX, state.TouchY);
		}

		// Recheck later to detect end of press
		uv_async_send(&asyncNotifyTouchIrq);
	} else if(state.TouchDetected == 0 && touchIsPressed) {
		touchIsPressed = false;
		uv_timer_start(&timerLcdOff, &LCDController::onTimerLcdOff, 30000, 0);
	}
}

void LCDController::onTimerLcdOff(uv_timer_t* handle) {
	// Touchscreen not pressed and LCD is ON
	// 30s elapsed => turn off LCD screen and backlight

	LCDController* thisInstance = (LCDController*) handle->data;

	if(!thisInstance->touchIsPressed && thisInstance->lcdIsOn) {
		thisInstance->lcdTurnOff();
	}
}

void LCDController::lcdTurnOn() {
	lcdIsOn = true;
	LCDDisplayOn();
}

void LCDController::lcdTurnOff() {
	lcdIsOn = false;
	LCDDisplayOff();
}

void LCDController::drawIcon(
    Icon icon, uint32_t color, int x, int y, int width, int height, TouchHandleCallback onClick) {
	switch(icon) {
		case ICON_ARROW_UP: {
			int pointsX[3] = {x + width / 2, x, x + width};
			int pointsY[3] = {y, y + height, y + height};

			LCDDrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1], color);
			LCDDrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2], color);
			LCDDrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2], color);
			break;
		}
		case ICON_ARROW_DOWN: {
			int pointsX[3] = {x + width / 2, x, x + width};
			int pointsY[3] = {y + height, y, y};

			LCDDrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1], color);
			LCDDrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2], color);
			LCDDrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2], color);
			break;
		}
		case ICON_ARROW_LEFT: {
			int pointsX[3] = {x, x + width, x + width};
			int pointsY[3] = {y + height / 2, y, y + height};

			LCDDrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1], color);
			LCDDrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2], color);
			LCDDrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2], color);
			break;
		}
		case ICON_ARROW_RIGHT: {
			int pointsX[3] = {x + width, x, x};
			int pointsY[3] = {y + height / 2, y, y + height};

			LCDDrawLine(pointsX[0], pointsY[0], pointsX[1], pointsY[1], color);
			LCDDrawLine(pointsX[1], pointsY[1], pointsX[2], pointsY[2], color);
			LCDDrawLine(pointsX[0], pointsY[0], pointsX[2], pointsY[2], color);
			break;
		}
		case ICON_ARROW_BACK: {
			int16_t xMiddle = x + width * 8 / 24;
			int16_t yMiddle = y + height / 2;
			int16_t yMidPoint1 = y + height / 3;
			int16_t yMidPoint2 = y + 2 * height / 3;
			LCDPoint arrowLeftPoints[] = {
			    {(int16_t) (x), (int16_t) (yMiddle)},
			    {(int16_t) (xMiddle), (int16_t) (y)},
			    {(int16_t) (xMiddle), (int16_t) (yMidPoint1)},
			    {(int16_t) (x + width), (int16_t) (yMidPoint1)},
			    {(int16_t) (x + width), (int16_t) (yMidPoint2)},
			    {(int16_t) (xMiddle), (int16_t) (yMidPoint2)},
			    {(int16_t) (xMiddle), (int16_t) (y + height)},
			    {(int16_t) (x), (int16_t) (yMiddle)},
			};
			LCDDrawPolygon(arrowLeftPoints, sizeof(arrowLeftPoints) / sizeof(arrowLeftPoints[0]), color);
			break;
		}
		case ICON_BUTTON_RELEASED: {
			LCDPoint arrowLeftPoints[] = {
			    {(int16_t) (x), (int16_t) (y)},
			    {(int16_t) (x), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y)},
			};
			LCDDrawPolygon(arrowLeftPoints, sizeof(arrowLeftPoints) / sizeof(arrowLeftPoints[0]), color);
			break;
		}
		case ICON_BUTTON_PRESSED:
			LCDPoint arrowLeftPoints[] = {
			    {(int16_t) (x), (int16_t) (y)},
			    {(int16_t) (x), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y + height)},
			    {(int16_t) (x + width), (int16_t) (y)},
			};
			LCDFillRect(x, y, width, height, color);
			LCDDrawPolygon(arrowLeftPoints, sizeof(arrowLeftPoints) / sizeof(arrowLeftPoints[0]), color);
			break;
	}
	if(onClick)
		touchHandlers.push_back({x, y, width, height, onClick});
}

void LCDController::clearScreen() {
	LCDFillScreen(LCD_UTILS_COLOR_BLACK);
}

void LCDController::drawHeader() {
	size_t numberOfColumnsDrawn = 0;
	size_t charWidth = LCDGetFont()->Width;

	for(size_t i = 0; i < menuHistorySize && numberOfColumnsDrawn < 240; i++) {
		MenuInfo* menu = &menuHistory[i];

		if(i > 0) {
			// Print separator
			LCDDisplayStringAt(numberOfColumnsDrawn, 0, LCD_UTILS_COLOR_WHITE, ">", LEFT_MODE);
			numberOfColumnsDrawn += 1 * charWidth;
		}

		LCDDisplayStringAt(numberOfColumnsDrawn, 0, LCD_UTILS_COLOR_WHITE, menu->name, LEFT_MODE);
		numberOfColumnsDrawn += strlen(menu->name) * charWidth;
	}

	if(menuHistorySize > 1) {
		drawIcon(ICON_ARROW_BACK, LCD_UTILS_COLOR_GREEN, 2, 8, 24, 16, nullptr);
		// Use hardcoded zone for a easier touch
		touchHandlers.push_back({0, 0, 120, 60, [](LCDController* thisInstance) { thisInstance->popMenu(); }});
	}
}

void LCDController::drawPanelButtonX1(PanelPosition panelIndex,
                                      const char* buttonName,
                                      bool buttonState,
                                      TouchHandleCallback buttonClick) {
	int16_t offset = panelIndex == PP_Left ? 0 : 120;
	drawIcon(buttonState ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED,
	         LCD_UTILS_COLOR_GREEN,
	         6 + offset,
	         114,
	         108,
	         60,
	         buttonClick);
	LCDDisplayStringAt(6 + 108 / 2 + offset, 114 + 60 / 2, LCD_UTILS_COLOR_WHITE, buttonName, CENTER_MODE);
}

void LCDController::drawPanelButtonX2(PanelPosition panelIndex,
                                      const char* button1Name,
                                      bool button1State,
                                      TouchHandleCallback button1Click,
                                      const char* button2Name,
                                      bool button2State,
                                      TouchHandleCallback button2Click) {
	int16_t offset = panelIndex == PP_Left ? 0 : 120;

	drawIcon(button1State ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED,
	         LCD_UTILS_COLOR_GREEN,
	         6 + offset,
	         60,
	         108,
	         60,
	         button1Click);
	drawIcon(button2State ? ICON_BUTTON_PRESSED : ICON_BUTTON_RELEASED,
	         LCD_UTILS_COLOR_GREEN,
	         6 + offset,
	         168,
	         108,
	         60,
	         button2Click);

	LCDDisplayStringAt(6 + 108 / 2 + offset, 60 + 60 / 2, LCD_UTILS_COLOR_WHITE, button1Name, CENTER_MODE);
	LCDDisplayStringAt(6 + 108 / 2 + offset, 168 + 60 / 2, LCD_UTILS_COLOR_WHITE, button2Name, CENTER_MODE);
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
	LCDPoint groupBoxPoints[] = {
	    {(int16_t) (6 + offset), (int16_t) (24)},
	    {(int16_t) (6 + offset), (int16_t) (24 + 210)},
	    {(int16_t) (114 + offset), (int16_t) (24 + 210)},
	    {(int16_t) (114 + offset), (int16_t) (24)},
	};
	LCDDrawPolygon(groupBoxPoints, sizeof(groupBoxPoints) / sizeof(groupBoxPoints[0]), LCD_UTILS_COLOR_WHITE);

	// Up arrow
	uint32_t color;
	if(arrowUpAvailable) {
		color = LCD_UTILS_COLOR_GREEN;
	} else {
		color = LCD_UTILS_COLOR_WHITE;
	}
	drawIcon(ICON_ARROW_UP, color, 6 + offset, 60, 108, 60, upClick);

	// Down arrow
	if(arrowDownAvailable) {
		color = LCD_UTILS_COLOR_GREEN;
	} else {
		color = LCD_UTILS_COLOR_WHITE;
	}
	drawIcon(ICON_ARROW_DOWN, color, 6 + offset, 168, 108, 60, downClick);

	// Title string
	LCDDisplayStringAt(6 + 108 / 2 + offset, 24 + 30 / 2, LCD_UTILS_COLOR_WHITE, title, CENTER_MODE);

	// Value string
	LCDDisplayStringAt(6 + 108 / 2 + offset, 129 + 30 / 2, LCD_UTILS_COLOR_WHITE, enumString, CENTER_MODE);
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
	LCDDrawPixel(touchX, touchY, 0x0000);
	touchX = x;
	touchY = y;
	LCDDrawPixel(touchX, touchY, 0xFFFF);

	for(const auto& touchHandler : touchHandlers) {
		if((x >= touchHandler.x && x <= touchHandler.x + touchHandler.width) &&
		   (y >= touchHandler.y && y <= touchHandler.y + touchHandler.height)) {
			touchHandler.handler(this);
			drawCurrentMenu();
		}
	}
}

#pragma once

#include <ChannelStrip.h>
#include <Osc/OscContainerArray.h>
#include <Osc/OscVariable.h>
#include <stdint.h>
#include <uv.h>

class OscRoot;

class LCDController {
public:
	LCDController(OscRoot* oscRoot);

	void start();

	void drawMenuMain();
	void drawMenuStripConfig();
	void notifyTouchFromIrq();

	static LCDController* instance;

protected:
	using TouchHandleCallback = void (*)(LCDController* thisInstance);
	using MenuHandlerCallback = void (LCDController::*)();

	enum Icon {
		ICON_ARROW_UP,
		ICON_ARROW_DOWN,
		ICON_ARROW_LEFT,
		ICON_ARROW_RIGHT,
		ICON_ARROW_BACK,
		ICON_BUTTON_RELEASED,
		ICON_BUTTON_PRESSED,
	};
	enum PanelType {
		PT_ButtonX1,
		PT_ButtonX2,
		PT_ConfigEnum,
		PT_ConfigFloat,
	};
	enum PanelPosition {
		PP_Left,
		PP_Right,
	};

	struct OscLink {
		PanelType type;
		union {
			struct {
				OscVariable<bool>* node;
			} buttonX1;
			struct {
				OscVariable<bool>* node[2];
			} buttonX2;
			struct {
				OscVariable<int32_t>* node;
			} configEnum;
			struct {
				OscVariable<float>* node;
			} configFloat;
		} data;
	};

	static void onTimerLcdOff(uv_timer_t* handle);
	static void onTouchIrqStatic(uv_async_t* handle);
	void onTouchIrq();

	void drawIcon(Icon icon, int x, int y, int width, int height, TouchHandleCallback onClick);

	void lcdTurnOn();
	void lcdTurnOff();
	void clearScreen();

	void drawHeader();
	void drawPanelButtonX1(PanelPosition panelIndex,
	                       const char* buttonName,
	                       bool buttonState,
	                       TouchHandleCallback buttonClick);
	void drawPanelButtonX2(PanelPosition panelIndex,
	                       const char* button1Name,
	                       bool button1State,
	                       TouchHandleCallback button1Click,
	                       const char* button2Name,
	                       bool button2State,
	                       TouchHandleCallback button2Click);
	void drawPanelConfigEnum(PanelPosition panelIndex,
	                         const char* title,
	                         const char* enumString,
	                         bool arrowUpAvailable,
	                         bool arrowDownAvailable,
	                         TouchHandleCallback upClick,
	                         TouchHandleCallback downClick);
	void drawPanelConfigFloat(PanelPosition panelIndex,
	                          const char* title,
	                          float value,
	                          const char* valueSuffix,
	                          TouchHandleCallback upClick,
	                          TouchHandleCallback downClick);

	void handleClick(int x, int y);

	void pushMenu(const char* menuName, MenuHandlerCallback menuHandler);
	void popMenu();
	void drawCurrentMenu();

	void updateStripDisplayString();

private:
	OscRoot* oscRoot;

	int touchX;
	int touchY;
	bool touchIsPressed;
	bool lcdIsOn;

	uv_timer_t timerLcdOff;
	uv_async_t asyncNotifyTouchIrq;

	// Screen metadata
	struct MenuInfo {
		const char* name;
		MenuHandlerCallback menuHandler;
	};
	MenuInfo menuHistory[10];
	size_t menuHistorySize;

	struct TouchHandler {
		int x;
		int y;
		int width;
		int height;
		TouchHandleCallback handler;
	};
	std::vector<TouchHandler> touchHandlers;

	// Menus state
	struct MenusState {
		size_t stripIndex;
		const char* stripDisplayName;

		OscReadOnlyVariable<bool>* muteNode;
		OscReadOnlyVariable<float>* volumeNode;

		size_t eqIndex;
		size_t compressorConfigIndex;
		size_t expanderConfigIndex;
	};
	MenusState menusState;
};

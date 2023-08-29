#pragma once

#include <ChannelStrip.h>
#include <Osc/OscContainerArray.h>
#include <Osc/OscVariable.h>
#include <stdint.h>

class OscRoot;

class LCDController {
public:
	LCDController(OscRoot* oscRoot);

	void start();

	void mainLoop();

	void drawMenuMain();
	void drawMenuStripConfig();

	static LCDController* instance;

protected:
	enum Icon {
		ICON_CONFIGURE,
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

	void drawIcon(Icon icon, int x, int y, int width, int height);

	void clearScreen();
	void drawHeader();
	void drawPanelButtonX1(PanelPosition panelIndex, const char* buttonName, bool buttonState);
	void drawPanelButtonX2(PanelPosition panelIndex,
	                       const char* button1Name,
	                       bool button1State,
	                       const char* button2Name,
	                       bool button2State);
	void drawPanelConfigEnum(PanelPosition panelIndex,
	                         const char* title,
	                         const char* enumString,
	                         bool arrowUpAvailable,
	                         bool arrowDownAvailable);
	void drawPanelConfigFloat(PanelPosition panelIndex, const char* title, float value, const char* valueSuffix);

	void handleClick(int x, int y);

	const char* getStripDisplayStringNode(size_t stripIndex);

	void pushMenu(const char* previousMenuName);
	void popMenu();

private:
	OscRoot* oscRoot;
	OscNode* oscStrips;

	int touchX;
	int touchY;
	bool touchIsPressed;

	// Screen metadata
	const char* menuPath[10];

	using TouchHandleCallback = void (*)(LCDController* thisInstance);
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

		OscReadOnlyVariable<bool>* muteNode;
		OscReadOnlyVariable<float>* volumeNode;

		size_t eqIndex;
		size_t compressorConfigIndex;
		size_t expanderConfigIndex;
	};
	MenusState menusState;

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
};

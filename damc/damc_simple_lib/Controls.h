#pragma once

#include <Osc/OscReadOnlyVariable.h>
#include <array>
#include <stdint.h>

class OscRoot;

class Controls {
public:
	Controls(OscRoot* oscRoot);

	void init();
	void mainLoop();

	// Called from USB interrupt
	uint16_t getControlFromUSB(uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest);
	void setControlFromUSB(
	    uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest, uint16_t value);

	static Controls* instance;

protected:
private:
	OscRoot* oscRoot;

	template<typename T> struct OscVariableChangeReq {
		T value;
		bool isChanged;
	};

	struct UsbOscControlMapping {
		OscReadOnlyVariable<float>* volumeControl;
		OscReadOnlyVariable<bool>* muteControl;
		OscVariableChangeReq<float> volumeToSet;
		OscVariableChangeReq<bool> muteToSet;
	};

	std::array<UsbOscControlMapping, 3> controlsMapping;
	bool processUsbControlChange;
};

#pragma once

#include <Osc/OscReadOnlyVariable.h>
#include <array>
#include <stdint.h>

class OscRoot;

class Controls {
public:
	Controls(OscRoot* oscRoot);

	void init();
	uint16_t getControlFromUSB(uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest);
	void setControlFromUSB(
	    uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest, uint16_t value);

	static Controls* instance;

protected:
private:
	OscRoot* oscRoot;

	std::array<OscReadOnlyVariable<float>*, 3> endpointVolumeControls;
	std::array<OscReadOnlyVariable<bool>*, 3> endpointMuteControls;
	bool processUsbControlChange;
};

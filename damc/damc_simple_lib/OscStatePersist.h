#pragma once

#include "ChannelStrip.h"
#include "Controls.h"
#include "OscSerialClient.h"
#include <FilteringChain.h>
#include <Osc/OscContainer.h>
#include <Osc/OscDynamicVariable.h>
#include <Osc/OscVariable.h>
#include <OscRoot.h>
#include <stdint.h>

class OscStatePersist : public OscContainer {
public:
	OscStatePersist(OscRoot* oscRoot);
	~OscStatePersist();

	void init();
	void mainLoop();

protected:
	void loadState();
	void saveState();
	void eraseSpi();
	void writeSpi(uint8_t* data, size_t size);
	void flushSpi();

private:
	OscRoot* oscRoot;
	OscVariable<int32_t> oscSaveConfigCount;
	OscDynamicVariable<bool> oscSaveNow;

	uint32_t oscConfigSaveTimerPreviousTick;
	bool oscConfigChanged;
	bool oscNeedSaveConfig;

	std::vector<uint8_t> buffer;
	size_t spiWriteAddress;
};

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
#include <uv.h>

class OscStatePersist : public OscContainer {
public:
	OscStatePersist(OscRoot* oscRoot);
	~OscStatePersist();

	void init();

protected:
	void loadState();
	void saveState();

	void beginWrite();
	void eraseNextBlockSpi();
	void writeSpiPage(uint8_t* data, size_t size);
	void writeMessage(uint8_t* data, size_t size);
	void flushSpi();

	static void autoSaveTimer(uv_timer_t* handle);

private:
	OscRoot* oscRoot;
	OscVariable<int32_t> oscSaveConfigCount;
	OscVariable<bool> oscSaveNow;
	std::function<void(std::string_view nodeFullName, uint8_t* data, size_t size)> saveSerializedMessage;

	bool oscConfigChanged;
	bool oscNeedSaveConfig;

	std::vector<uint8_t> buffer;
	size_t spiWriteAddress;
	size_t spiErasedAddress;

	uv_timer_t timerAutoSave;
};

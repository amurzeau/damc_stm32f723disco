#include "OscStatePersist.h"
#include "OscRoot.h"
#include "TimeMeasure.h"

#include <stm32f723e_discovery_qspi.h>

OscStatePersist::OscStatePersist(OscRoot* oscRoot)
    : OscContainer(oscRoot, "config"),
      oscRoot(oscRoot),
      oscSaveConfigCount(this, "saveCount"),
      oscSaveNow(this, "saveNow"),
      oscConfigSaveTimerPreviousTick(0),
      oscConfigChanged(false),
      oscNeedSaveConfig(false) {}

OscStatePersist::~OscStatePersist() {}

void OscStatePersist::init() {
	BSP_QSPI_Init();

	// Load default config
	using namespace std::literals;
	const std::map<std::string_view, std::vector<OscArgument>> default_config = {
	    {"/strip/1/filterChain/compressorFilter/makeUpGain", {float{-1.f}}},
	    {"/strip/0/display_name", {"master"sv}},
	    {"/strip/1/display_name", {"comp"sv}},
	    {"/strip/2/display_name", {"mic"sv}},
	    {"/strip/3/display_name", {"out-record"sv}},
	    {"/strip/4/display_name", {"mic-feedback"sv}},
	    {"/strip/0/filterChain/volume", {float{-25.f}}},
	    {"/strip/1/filterChain/compressorFilter/enable", {true}},
	    {"/strip/3/filterChain/mute", {true}},
	    {"/strip/4/filterChain/mute", {true}},
	};

	oscRoot->loadNodeConfig(default_config);

	// Load stored config
	loadState();

	// Listen for config changes
	oscRoot->setOnOscValueChanged([this]() { oscConfigChanged = true; });
	oscSaveNow.setWriteCallback([this](bool value) {
		if(value) {
			saveState();
		}
	});
}

void OscStatePersist::beginWrite() {
	buffer.clear();
	spiWriteAddress = 0;
	spiErasedAddress = 0;
}

void OscStatePersist::eraseNextBlockSpi() {
	// Erase 4KB
	BSP_QSPI_Erase_Block(spiErasedAddress);
	spiErasedAddress += 4096;
}

uint16_t checksum(uint8_t* data, size_t size) {
	uint16_t sum = 0;
	for(size_t i = 0; i < size; i++) {
		sum += data[i];
	}
	return sum;
}

void OscStatePersist::writeSpiPage(uint8_t* data, size_t size) {
	// Erase blocks that need to be written
	while(spiWriteAddress + size > spiErasedAddress) {
		eraseNextBlockSpi();
	}
	// Write page
	BSP_QSPI_Write(data, spiWriteAddress, size);
	spiWriteAddress += size;
}

void OscStatePersist::writeMessage(uint8_t* data, size_t size) {
	size_t i;
	uint16_t sum = checksum(data, size);
	uint8_t messageSize[] = {
	    (uint8_t) (size & 0xFF),
	    (uint8_t) ((size >> 8) & 0xFF),
	    (uint8_t) (sum & 0xFF),
	    (uint8_t) ((sum >> 8) & 0xFF),
	};

	// Add message size
	buffer.insert(buffer.end(), messageSize, messageSize + sizeof(messageSize));

	// Add message data
	buffer.insert(buffer.end(), data, data + size);

	// Write completed pages to SPI flash
	for(i = 0; i + 256 <= buffer.size(); i += 256) {
		writeSpiPage(&buffer[i], 256);
	}

	// Remove all written data
	buffer.erase(buffer.begin(), buffer.begin() + i);
}

void OscStatePersist::flushSpi() {
	// Write null message to mark end of data
	writeMessage(nullptr, 0);

	// Write remaining buffer data
	if(buffer.size() > 0) {
		writeSpiPage(&buffer[0], buffer.size());
		buffer.clear();
	}
}

void OscStatePersist::loadState() {
	uint8_t readData[1024] = {0};
	buffer.clear();

	for(size_t spiReadAddress = 0; spiReadAddress < 8 * 4096;) {
		BSP_QSPI_Read(readData, spiReadAddress, 4);
		spiReadAddress += 4;

		uint32_t messageSize = readData[0] | (readData[1] << 8);
		uint32_t sum = readData[2] | (readData[3] << 8);
		if(messageSize == 0) {
			// Invalid data
			break;
		}
		if(messageSize == 0xFFFF) {
			// Flash is not programmed here, abort loading
			break;
		}

		if(messageSize < 1024) {
			BSP_QSPI_Read(readData, spiReadAddress, messageSize);
			uint16_t readSum = checksum(readData, messageSize);
			if(sum != readSum) {
				break;
			}

			oscRoot->onOscPacketReceived(readData, messageSize);
		}
		spiReadAddress += messageSize;
	}
}

void OscStatePersist::saveState() {
	oscSaveConfigCount.set(oscSaveConfigCount.get() + 1);
	oscNeedSaveConfig = false;
	oscConfigChanged = false;

	beginWrite();

	// Store number of saves along with other configs
	OscArgument argument = oscSaveConfigCount.getToOsc();
	oscRoot->serializeMessage(
	    [this](std::string_view nodeFullName, uint8_t* data, size_t size) { writeMessage(data, size); },
	    &oscSaveConfigCount,
	    &argument,
	    1);

	oscRoot->visit([this](OscNode* node, OscArgument* arguments, size_t number) {
		oscRoot->serializeMessage(
		    [this](std::string_view nodeFullName, uint8_t* data, size_t size) { writeMessage(data, size); },
		    node,
		    arguments,
		    number);
	});

	flushSpi();
}

void OscStatePersist::mainLoop() {
	uint32_t currentTick = HAL_GetTick();

	if(currentTick >= oscConfigSaveTimerPreviousTick + 10000) {
		TimeMeasure::timeMeasure[TMI_MainLoop].beginMeasure();

		oscConfigSaveTimerPreviousTick = currentTick;

		if(oscConfigChanged) {
			oscNeedSaveConfig = true;
			oscConfigChanged = false;
		} else if(oscNeedSaveConfig) {
			saveState();
		}

		TimeMeasure::timeMeasure[TMI_MainLoop].endMeasure();
	}
}
#include "OscStatePersist.h"
#include "OscRoot.h"
#include "TimeMeasure.h"

#ifdef STM32F723xx
#include <stm32f723e_discovery_qspi.h>

static void MEMORY_init() {
	BSP_QSPI_Init();
}

static void MEMORY_erase_block(size_t address) {
	BSP_QSPI_Erase_Block(address);
}

static void MEMORY_write(uint8_t* data, size_t address, size_t size) {
	BSP_QSPI_Write(data, address, size);
}

static void MEMORY_read(uint8_t* data, size_t address, size_t size) {
	BSP_QSPI_Read(data, address, size);
}
#elif defined(STM32N657xx)
#include <stm32n6570_discovery_xspi.h>

static void MEMORY_init() {
	return;
	BSP_XSPI_NOR_Init_t init = {
	    .InterfaceMode = MX66UW1G45G_SPI_MODE,
	    .TransferRate = MX66UW1G45G_STR_TRANSFER,
	};
	BSP_XSPI_NOR_Init(0, &init);
}

// 0x4000000: 128MB offset
static void MEMORY_erase_block(size_t address) {
	return;
	BSP_XSPI_NOR_Erase_Block(0, address + 0x4000000, MX66UW1G45G_ERASE_4K);
}

static void MEMORY_write(uint8_t* data, size_t address, size_t size) {
	return;
	BSP_XSPI_NOR_Write(0, data, address + 0x4000000, size);
}

static void MEMORY_read(uint8_t* data, size_t address, size_t size) {
	memset(data, 0xFF, size);
	return;
	BSP_XSPI_NOR_Read(0, data, address + 0x4000000, size);
}
#endif

OscStatePersist::OscStatePersist(OscRoot* oscRoot)
    : OscContainer(oscRoot, "config"),
      oscRoot(oscRoot),
      oscSaveConfigCount(this, "saveCount"),
      oscSaveNow(this, "saveNow", false, false),
      oscConfigChanged(false),
      oscNeedSaveConfig(false) {
	saveSerializedMessage = [this](std::string_view nodeFullName, uint8_t* data, size_t size) {
		writeMessage(data, size);
	};
	uv_timer_init(uv_default_loop(), &timerAutoSave);
	timerAutoSave.data = this;
}

OscStatePersist::~OscStatePersist() {}

void OscStatePersist::init() {
	MEMORY_init();

	// Load default config
	using namespace std::literals;
	const std::map<std::string_view, std::vector<OscArgument>> default_config = {
	    {"/strip/0/filterChain/volume", {float{-25.f}}},
	    {"/strip/1/filterChain/compressorFilter/enable", {true}},
	    {"/strip/1/filterChain/compressorFilter/makeUpGain", {float{-1.f}}},
	    {"/strip/3/filterChain/mute", {true}},
	    {"/strip/4/filterChain/mute", {true}},
	};

	oscRoot->loadNodeConfig(default_config);

	// Load stored config
	loadState();

	// Listen for config changes
	oscRoot->addValueChangedCallback([this]() {
		oscConfigChanged = true;
		;
	});
	oscSaveNow.addChangeCallback([this](bool value) {
		if(value) {
			saveState();
		}
	});

	uv_timer_start(&timerAutoSave, &OscStatePersist::autoSaveTimer, 10000, 10000);
}

void OscStatePersist::beginWrite() {
	buffer.clear();
	spiWriteAddress = 0;
	spiErasedAddress = 0;
}

void OscStatePersist::eraseNextBlockSpi() {
	// Erase 4KB
	MEMORY_erase_block(spiErasedAddress);
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
	MEMORY_write(data, spiWriteAddress, size);
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
		MEMORY_read(readData, spiReadAddress, 4);
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
			MEMORY_read(readData, spiReadAddress, messageSize);
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
	oscRoot->serializeMessage(saveSerializedMessage, &oscSaveConfigCount, &argument, 1);

	oscRoot->visit([this](OscNode* node, OscArgument* arguments, size_t number) {
		oscRoot->serializeMessage(saveSerializedMessage, node, arguments, number);
	});

	flushSpi();
}

void OscStatePersist::autoSaveTimer(uv_timer_t* handle) {
	OscStatePersist* thisInstance = (OscStatePersist*) handle->data;

	if(thisInstance->oscConfigChanged) {
		thisInstance->oscNeedSaveConfig = true;
		thisInstance->oscConfigChanged = false;
	} else if(thisInstance->oscNeedSaveConfig) {
		thisInstance->saveState();
	}
}
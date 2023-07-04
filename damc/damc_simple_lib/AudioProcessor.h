#pragma once

#include "ChannelStrip.h"
#include "OscSerialClient.h"
#include <FilteringChain.h>
#include <Osc/OscReadOnlyVariable.h>
#include <Osc/OscDynamicVariable.h>
#include <OscRoot.h>
#include <stdint.h>

class AudioProcessor {
public:
	AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes);
	~AudioProcessor();

	void processAudioInterleaved(int16_t index, const int16_t* data_input, int16_t* data_output, size_t nframes);
	void mainLoop();

	static AudioProcessor* getInstance();

private:
	OscRoot oscRoot;
	OscSerialClient serialClient;
	OscContainerArray<ChannelStrip> strips;

	OscReadOnlyVariable<int32_t> timeMeasureUsbInterrupt;
	OscReadOnlyVariable<int32_t> timeMeasureAudioProcessing;
	OscReadOnlyVariable<int32_t> timeMeasureFastTimer;
	OscReadOnlyVariable<int32_t> timeMeasureOscInput;

	OscReadOnlyVariable<int32_t> timeMeasureMaxPerLoopUsbInterrupt;
	OscReadOnlyVariable<int32_t> timeMeasureMaxPerLoopAudioProcessing;
	OscReadOnlyVariable<int32_t> timeMeasureMaxPerLoopFastTimer;
	OscReadOnlyVariable<int32_t> timeMeasureMaxPerLoopOscInput;

	OscDynamicVariable<int32_t> memoryAvailable;
	OscDynamicVariable<int32_t> memoryUsed;

	uint32_t fastTimerPreviousTick;
	uint32_t nextTimerStripIndex;
	uint32_t slowTimerPreviousTick;
	uint32_t slowTimerIndex;
};

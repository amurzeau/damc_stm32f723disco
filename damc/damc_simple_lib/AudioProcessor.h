#pragma once

#include "ChannelStrip.h"
#include "Controls.h"
#include "OscSerialClient.h"
#include <FilteringChain.h>
#include <Osc/OscDynamicVariable.h>
#include <Osc/OscReadOnlyVariable.h>
#include <OscRoot.h>
#include <OscStatePersist.h>
#include <stdint.h>

class MultiChannelAudioBuffer {
public:
	MultiChannelAudioBuffer();

	static constexpr size_t CHANNEL_NUMBER = 2;
	static constexpr size_t BUFFER_SIZE = 48 * 2;

	float data[CHANNEL_NUMBER][BUFFER_SIZE];
	float* dataPointers[CHANNEL_NUMBER];
};

class AudioProcessor {
public:
	AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes);
	~AudioProcessor();

	void processAudioInterleaved(const int16_t** input_endpoints,
	                             size_t input_endpoints_number,
	                             int16_t** output_endpoints,
	                             size_t output_endpoints_number,
	                             size_t nframes);
	void mainLoop();

	Controls* getControls() { return &controls; }

	static AudioProcessor* getInstance();

protected:
	void interleavedToFloat(const int16_t* data_input, MultiChannelAudioBuffer* data_float, size_t nframes);
	void floatToInterleaved(MultiChannelAudioBuffer* data_float, int16_t* data_output, size_t nframes);
	void mixAudio(MultiChannelAudioBuffer* mixed_data, MultiChannelAudioBuffer* data_to_add, size_t nframes);

private:
	uint32_t numChannels;

	OscRoot oscRoot;
	OscSerialClient serialClient;
	Controls controls;
	OscContainerArray<ChannelStrip> strips;
	OscStatePersist oscStatePersist;

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

	MultiChannelAudioBuffer buffer[4];
	int16_t codecBuffer[MultiChannelAudioBuffer::BUFFER_SIZE * MultiChannelAudioBuffer::CHANNEL_NUMBER]
	    __attribute__((aligned(4)));
};

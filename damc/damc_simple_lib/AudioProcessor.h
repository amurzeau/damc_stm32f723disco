#pragma once

#include "ChannelStrip.h"
#include "Controls.h"
#include "LCDController.h"
#include "OscSerialClient.h"
#include "TimeMeasure.h"
#include <FilteringChain.h>
#include <Osc/OscContainerArray.h>
#include <Osc/OscDynamicVariable.h>
#include <Osc/OscFixedArray.h>
#include <Osc/OscReadOnlyVariable.h>
#include <OscRoot.h>
#include <OscStatePersist.h>
#include <array>
#include <stdint.h>

class MultiChannelAudioBuffer {
public:
	MultiChannelAudioBuffer();

	static constexpr size_t CHANNEL_NUMBER = 2;
	static constexpr size_t BUFFER_SIZE = 48;

	float data[CHANNEL_NUMBER][BUFFER_SIZE];
	float* dataPointers[CHANNEL_NUMBER];
};

class AudioProcessor {
public:
	AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes);
	~AudioProcessor();

	void start();

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
	void interleavedToFloatCodec(const int16_t* data_input, MultiChannelAudioBuffer* data_float0, size_t nframes);
	void floatToInterleavedCodec(MultiChannelAudioBuffer* data_float0,
	                             MultiChannelAudioBuffer* data_float1,
	                             int16_t* data_output,
	                             size_t nframes);
	void mixAudio(MultiChannelAudioBuffer* mixed_data, MultiChannelAudioBuffer* data_to_add, size_t nframes);

private:
	uint32_t numChannels;

	OscRoot oscRoot;
	OscSerialClient serialClient;
	Controls controls;
	OscFixedArray<ChannelStrip, 6> oscStrips;
	std::array<ChannelStrip, 6> strips;
	OscStatePersist oscStatePersist;

	OscReadOnlyVariable<int32_t> oscTimeMeasure[TMI_NUMBER];
	OscReadOnlyVariable<int32_t> oscTimeMeasureMaxPerLoop[TMI_NUMBER];

	OscDynamicVariable<int32_t> fastMemoryAvailable;
	OscDynamicVariable<int32_t> fastMemoryUsed;
	OscDynamicVariable<int32_t> slowMemoryAvailable;
	OscDynamicVariable<int32_t> slowMemoryUsed;

	uint32_t fastTimerPreviousTick;
	uint32_t nextTimerStripIndex;
	uint32_t slowTimerPreviousTick;
	uint32_t slowTimerIndex;

	MultiChannelAudioBuffer buffer[5];
	int16_t codecBuffer[MultiChannelAudioBuffer::BUFFER_SIZE * MultiChannelAudioBuffer::CHANNEL_NUMBER * 2]
	    __attribute__((aligned(4)));

	LCDController lcdController;
};

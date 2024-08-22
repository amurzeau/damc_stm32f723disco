#pragma once

#include "ChannelStrip.h"
#include "CodecAudio.h"
#include "Controls.h"
#include "LCDController.h"
#include "OscSerialClient.h"
#include "TimeMeasure.h"
#include <FilteringChain.h>
#include <Osc/OscContainerArray.h>
#include <Osc/OscFixedArray.h>
#include <Osc/OscReadOnlyVariable.h>
#include <Osc/OscVariable.h>
#include <OscRoot.h>
#include <OscStatePersist.h>
#include <array>
#include <stdint.h>
#include <uv.h>

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

	Controls* getControls() { return &controls; }

	static AudioProcessor* getInstance();

protected:
	void interleavedToFloat(const int16_t* data_input, MultiChannelAudioBuffer* data_float, size_t nframes);
	void floatToInterleaved(MultiChannelAudioBuffer* data_float, int16_t* data_output, size_t nframes);
	void interleavedToFloatCodec(const CodecAudio::CodecFrame* data_input,
	                             MultiChannelAudioBuffer* data_float0,
	                             size_t nframes);
	void floatToInterleavedCodec(MultiChannelAudioBuffer* data_float,
	                             CodecAudio::CodecFrame* data_output,
	                             size_t nframes);
	void mixAudio(MultiChannelAudioBuffer* mixed_data, MultiChannelAudioBuffer* data_to_add, size_t nframes);
	void mixAudioStereoToDualMono(MultiChannelAudioBuffer* dual_mono, MultiChannelAudioBuffer* output2, size_t nframes);

	static void onFastTimer(uv_timer_t* handle);
	static void onSlowTimer(uv_timer_t* handle);

	void startFastTimer();
	void startSlowTimer();

private:
	MultiChannelAudioBuffer buffer[5];
	CodecAudio::CodecFrame codecBuffer[MultiChannelAudioBuffer::BUFFER_SIZE * 2] __attribute__((aligned(4)));

	uint32_t numChannels;

	OscRoot oscRoot;
	OscFixedArray<ChannelStrip, 6> oscStrips;
	std::array<ChannelStrip, 6> strips;
	OscStatePersist oscStatePersist;

	OscReadOnlyVariable<int32_t> oscTimeMeasure[TMI_NUMBER];
	OscReadOnlyVariable<int32_t> oscTimeMeasureMaxPerLoop[TMI_NUMBER];
	OscVariable<bool> oscEnableMicBias;

	OscReadOnlyVariable<int32_t> fastMemoryAvailable;
	OscReadOnlyVariable<int32_t> fastMemoryUsed;
	OscReadOnlyVariable<int32_t> slowMemoryAvailable;
	OscReadOnlyVariable<int32_t> slowMemoryUsed;

	uint32_t nextTimerStripIndex;
	uint32_t slowTimerIndex;

	LCDController lcdController;
	OscSerialClient serialClient;
	Controls controls;

	uv_timer_t timerFastStrips;
	uv_timer_t timerSlowMeasures;
};

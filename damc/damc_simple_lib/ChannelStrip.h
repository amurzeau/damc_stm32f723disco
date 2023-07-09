#pragma once

#include <FilteringChain.h>
#include <Osc/OscContainer.h>
#include <Osc/OscReadOnlyVariable.h>
#include <stdint.h>

class ChannelStrip : public OscContainer {
public:
	ChannelStrip(OscContainer* parent, int index, std::string_view name, uint32_t numChannels, uint32_t sampleRate, size_t maxNframes);
	~ChannelStrip();

	void processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes);
	void processSamples(float** samples, size_t numChannel, size_t nframes);
	void onFastTimer();

private:
	OscVariable<bool> oscEnable;
	OscVariable<int32_t> oscType;
	OscVariable<std::string> oscName;
	OscVariable<std::string> oscDisplayName;
	OscReadOnlyVariable<int32_t> oscNumChannels;
	OscReadOnlyVariable<int32_t> oscSampleRate;
	FilterChain filterChain;

	size_t maxNframes;
	float** channelBuffers;
};

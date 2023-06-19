#pragma once

#include "OscSerialClient.h"
#include <FilteringChain.h>
#include <Osc/OscReadOnlyVariable.h>
#include <OscRoot.h>
#include <stdint.h>

class AudioProcessor {
public:
	AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes);
	~AudioProcessor();

	void processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes);
	void mainLoop();

	static AudioProcessor* getInstance();

private:
	OscRoot oscRoot;
	OscSerialClient serialClient;
	OscReadOnlyVariable<int32_t> oscNumChannels;
	OscReadOnlyVariable<int32_t> oscSampleRate;
	FilterChain filterChain;

	size_t maxNframes;
	float** channelBuffers;

};

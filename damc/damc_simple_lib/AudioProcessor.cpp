#include "AudioProcessor.h"
#include "OscSerialClient.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <spdlog/spdlog.h>

AudioProcessor* AudioProcessor::getInstance() {
	static AudioProcessor instance(2, 48000, 48);
	return &instance;
}

AudioProcessor::AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes)
    : oscRoot(true),
      serialClient(&oscRoot),
      oscNumChannels(&oscRoot, "numChannels", numChannels),
      oscSampleRate(&oscRoot, "sampleRate", sampleRate),
      filterChain(&oscRoot, &oscNumChannels, &oscSampleRate),
      maxNframes(maxNframes) {

	oscNumChannels.addCheckCallback([](int32_t value) -> bool { return false; });
	oscSampleRate.addCheckCallback([](int32_t value) -> bool { return false; });

	channelBuffers = new float*[numChannels];

	for(size_t i = 0; i < numChannels; i++) {
		channelBuffers[i] = new float[maxNframes];
	}

	serialClient.init();
}

AudioProcessor::~AudioProcessor() {
	for(int i = 0; i < oscNumChannels.get(); i++) {
		delete[] channelBuffers[i];
	}
	delete[] channelBuffers;
	channelBuffers = nullptr;
}

void AudioProcessor::processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes) {
	uint32_t channelNumber = (uint32_t) oscNumChannels.get();

	if(nframes > maxNframes) {
		return;
	}

	for(uint32_t channel = 0; channel < channelNumber; channel++) {
		for(uint32_t frame = 0; frame < nframes; frame++) {
			channelBuffers[channel][frame] = data_input[frame * channelNumber + channel] / 32768.f;
		}
	}

	filterChain.processSamples(channelBuffers, channelNumber, nframes);

	for(uint32_t channel = 0; channel < channelNumber; channel++) {
		for(uint32_t frame = 0; frame < nframes; frame++) {
			data_output[frame * channelNumber + channel] = static_cast<int16_t>(channelBuffers[channel][frame] * 32768.f);
		}
	}
}

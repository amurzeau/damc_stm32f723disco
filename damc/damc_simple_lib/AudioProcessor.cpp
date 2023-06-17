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

	float** channelBuffers = new float*[numChannels];

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

void AudioProcessor::processAudioInterleaved(int16_t* data, size_t nframes) {
	size_t channelNumber = oscNumChannels.get();

	if(nframes > maxNframes) {
		return;
	}

	for(size_t frame = 0; frame < nframes; frame++) {
		for(size_t channel = 0; channel < channelNumber; channel++) {
			channelBuffers[channel][frame] = data[frame * channelNumber + channel] / 32768.f;
		}
	}

	filterChain.processSamples(channelBuffers, (const float**) channelBuffers, channelNumber, nframes);

	for(size_t frame = 0; frame < nframes; frame++) {
		for(size_t channel = 0; channel < channelNumber; channel++) {
			data[frame * channelNumber + channel] = static_cast<int16_t>(channelBuffers[channel][frame] * 32768.f);
		}
	}
}

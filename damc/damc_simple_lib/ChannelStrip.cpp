#include "ChannelStrip.h"
#include "OscSerialClient.h"
#include <Utils.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <time.h>

#include <spdlog/spdlog.h>

ChannelStrip::ChannelStrip(
    OscContainer* parent, int index, std::string_view name, uint32_t numChannels, uint32_t sampleRate, size_t maxNframes)
    : OscContainer(parent, Utils::toString(index), 10),
      oscEnable(this, "enable", true),
      oscType(this, "_type", 0),
      oscName(this, "name", Utils::toString(index)),
      oscDisplayName(this, "display_name", name),
      oscNumChannels(this, "channels", numChannels),
      oscSampleRate(this, "sample_rate", sampleRate),
      filterChain(this, &oscNumChannels, &oscSampleRate),
      maxNframes(maxNframes) {
	oscNumChannels.addCheckCallback([](int32_t value) -> bool { return false; });
	oscSampleRate.addCheckCallback([](int32_t value) -> bool { return false; });

	channelBuffers = new float*[numChannels];

	for(size_t i = 0; i < numChannels; i++) {
		channelBuffers[i] = new float[maxNframes];
	}
}

ChannelStrip::~ChannelStrip() {
	for(int i = 0; i < oscNumChannels.get(); i++) {
		delete[] channelBuffers[i];
	}
	delete[] channelBuffers;
	channelBuffers = nullptr;
}

void ChannelStrip::processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes) {
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
			data_output[frame * channelNumber + channel] =
			    static_cast<int16_t>(channelBuffers[channel][frame] * 32768.f);
		}
	}
}

void ChannelStrip::processSamples(float** samples, size_t numChannel, size_t nframes) {
	filterChain.processSamples(samples, numChannel, nframes);
}

void ChannelStrip::onFastTimer() {
	filterChain.onFastTimer();
}

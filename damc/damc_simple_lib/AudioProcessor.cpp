#include "AudioProcessor.h"
#include "OscSerialClient.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <spdlog/spdlog.h>

#include <stm32f7xx_hal.h>
#include <usbd_conf.h>

AudioProcessor* AudioProcessor::getInstance() {
	static AudioProcessor instance(2, 48000, 48);
	return &instance;
}

volatile uint32_t variable = sizeof(ChannelStrip);

AudioProcessor::AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes)
    : oscRoot(true), serialClient(&oscRoot), strips(&oscRoot, "strip") {
	strips.setFactory([this, numChannels, sampleRate, maxNframes](OscContainer* parent, int name) {
		return new ChannelStrip(parent, name, numChannels, sampleRate, maxNframes);
	});

	strips.resize(AUDIO_LOOPBACKS_NUMBER);

	serialClient.init();
}

AudioProcessor::~AudioProcessor() {}

void AudioProcessor::processAudioInterleaved(int16_t index,
                                             const int16_t* data_input,
                                             int16_t* data_output,
                                             size_t nframes) {
	if(index < AUDIO_LOOPBACKS_NUMBER)
		strips.at(index).processAudioInterleaved(data_input, data_output, nframes);
	else {
		memcpy(data_output, data_input, nframes * USBD_AUDIO_BYTES_PER_SAMPLE * USBD_AUDIO_CHANNELS);
	}
}

void AudioProcessor::mainLoop() {
	serialClient.mainLoop();
	uint32_t currentTick = HAL_GetTick();
	if(currentTick >= previousTick + 100) {
		// Every 100ms
		previousTick = currentTick;
		for(auto& strip : strips) {
			strip.second->onFastTimer();
		}
	}
}

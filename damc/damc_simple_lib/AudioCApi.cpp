#include "AudioCApi.h"
#include "AudioProcessor.h"

void DAMC_init() {
	// This will allocate instance
	AudioProcessor::getInstance();
}

void DAMC_processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes) {
	AudioProcessor::getInstance()->processAudioInterleaved(data_input, data_output, nframes);
}

void DAMC_mainLoop() {
	AudioProcessor::getInstance()->mainLoop();
}

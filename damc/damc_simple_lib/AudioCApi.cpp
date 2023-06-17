#include "AudioCApi.h"
#include "AudioProcessor.h"

void processAudioInterleaved(int16_t* data, size_t nframes) {
	AudioProcessor::getInstance()->processAudioInterleaved(data, nframes);
}

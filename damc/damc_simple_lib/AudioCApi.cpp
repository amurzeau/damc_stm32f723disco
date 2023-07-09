#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "TimeMeasure.h"
#include "CodecAudio.h"

void DAMC_init() {
	// This will allocate instance
	AudioProcessor::getInstance();
	TimeMeasure::updateClockPerUs();
}

void DAMC_start() {
	AudioProcessor::getInstance()->init();
	CodecAudio::instance.start();
}

void DAMC_processAudioInterleaved(const int16_t** input_endpoints, size_t input_endpoints_number, int16_t** output_endpoints, size_t output_endpoints_number, size_t nframes) {
	AudioProcessor::getInstance()->processAudioInterleaved(input_endpoints, input_endpoints_number, output_endpoints, output_endpoints_number, nframes);
}

void DAMC_mainLoop() {
	AudioProcessor::getInstance()->mainLoop();
	CodecAudio::instance.onFastTimer();
}

void DAMC_usbInterruptBeginMeasure() {
	TimeMeasure::timeMeasureUsbInterrupt.beginMeasure();
}

void DAMC_usbInterruptEndMeasure() {
	TimeMeasure::timeMeasureUsbInterrupt.endMeasure();
}

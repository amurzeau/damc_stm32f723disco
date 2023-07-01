#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "TimeMeasure.h"

void DAMC_init() {
	// This will allocate instance
	AudioProcessor::getInstance();
	TimeMeasure::updateClockPerUs();
}

void DAMC_processAudioInterleaved(int16_t index, const int16_t* data_input, int16_t* data_output, size_t nframes) {
	AudioProcessor::getInstance()->processAudioInterleaved(index, data_input, data_output, nframes);
}

void DAMC_mainLoop() {
	AudioProcessor::getInstance()->mainLoop();
}

void DAMC_usbInterruptBeginMeasure() {
	TimeMeasure::timeMeasureUsbInterrupt.beginMeasure();
}

void DAMC_usbInterruptEndMeasure() {
	TimeMeasure::timeMeasureUsbInterrupt.endMeasure();
}

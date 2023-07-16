#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "CodecAudio.h"
#include "TimeMeasure.h"

void DAMC_init() {
	// This will allocate instance
	AudioProcessor::getInstance();
	TimeMeasure::updateClockPerUs();
}

void DAMC_start() {
	CodecAudio::instance.start();
	AudioProcessor::getInstance()->init();
}

void DAMC_processAudioInterleaved(const int16_t** input_endpoints,
                                  size_t input_endpoints_number,
                                  int16_t** output_endpoints,
                                  size_t output_endpoints_number,
                                  size_t nframes) {
	AudioProcessor::getInstance()->processAudioInterleaved(
	    input_endpoints, input_endpoints_number, output_endpoints, output_endpoints_number, nframes);
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

void DAMC_setControlFromUSB(uint8_t unit_id, uint8_t control_type, uint8_t channel, uint8_t bRequest, uint16_t value) {
	AudioProcessor::getInstance()->getControls()->setControlFromUSB(unit_id, control_type, channel, bRequest, value);
}

uint16_t DAMC_getControlFromUSB(uint8_t unit_id, uint8_t control_type, uint8_t channel, uint8_t bRequest) {
	return AudioProcessor::getInstance()->getControls()->getControlFromUSB(unit_id, control_type, channel, bRequest);
}

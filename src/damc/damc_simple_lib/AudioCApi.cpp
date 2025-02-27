#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "CodecAudio.h"
#include "GlitchDetection.h"
#include "TimeMeasure.h"
#include "Tracing.h"
#include "USBAudioBuffers.h"
#include "usbd_conf.h"
#include <string.h>
#include <usbd_audio.h>
#include <uv.h>

void DAMC_init() {
	// This will allocate instance
	AudioProcessor::getInstance();
}

void DAMC_start() {
	CodecAudio::instance.start();
	AudioProcessor::getInstance()->start();
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
	uv_run(uv_default_loop(), UV_RUN_DEFAULT);
}

void DAMC_beginMeasure(enum TimeMeasureItem item) {
	TimeMeasure::timeMeasure[item].beginMeasure();
}

void DAMC_endMeasure(enum TimeMeasureItem item) {
	TimeMeasure::timeMeasure[item].endMeasure();
}

void DAMC_resetFrequencyToMaxPerformance() {
	AudioProcessor::getInstance()->resetFrequencyToMaxPerformance();
}

void DAMC_setControlFromUSB(
    uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest, uint16_t value) {
	AudioProcessor::getInstance()->getControls()->setControlFromUSB(
	    unit_id, control_selector, channel, bRequest, value);
}

uint16_t DAMC_getControlFromUSB(uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest) {
	return AudioProcessor::getInstance()->getControls()->getControlFromUSB(
	    unit_id, control_selector, channel, bRequest);
}

static void DAMC_checkAudioInterruptLost() {
	static uint32_t previousInterruptTimestamp;

	uint32_t currentTimestamp = TimeMeasure::getCurrent();

	bool isInterruptLost = previousInterruptTimestamp && (currentTimestamp - previousInterruptTimestamp) > 1500;
	previousInterruptTimestamp = currentTimestamp;
	if(isInterruptLost) {
		GLITCH_DETECTION_increment_counter(GT_AudioProcessInterruptLost);
	}
}

/**
 * @brief Tx Transfer Half completed callback.
 * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval None
 */
static void DAMC_processAudioFromDMAInterrupt() {
	TRACING_add(false, 0, "Audio Processing start");

	// Ensure PSRAM is not acceeded while in audio interrupt
	// MPU_Config(0);
	TimeMeasure::on1msElapsed();

	// Check lost interrupt if more than 1.5ms elapsed
	DAMC_checkAudioInterruptLost();

	size_t nframes = 48;
	static uint32_t usb_buffers[3][48];

	const int16_t* endpoint_out_buffer[] = {
	    (const int16_t*) &usb_buffers[DUB_Out1],
	    (const int16_t*) &usb_buffers[DUB_Out2],
	};

	int16_t* endpoint_in_buffer[] = {
	    (int16_t*) &usb_buffers[DUB_In],
	};

	for(size_t i = 0; i < AUDIO_OUT_NUMBER; i++) {
		size_t readSize = usbBuffers[DUB_Out1 + i].readAudioSample(true, &usb_buffers[DUB_Out1 + i], nframes);
		if(USBD_AUDIO_IsEndpointEnabled(false, i) && readSize != nframes) {
			GLITCH_DETECTION_increment_counter(GT_UsbOutUnderrun);
		}
	}

	AudioProcessor::getInstance()->processAudioInterleaved((const int16_t**) endpoint_out_buffer,
	                                                       sizeof(endpoint_out_buffer) / sizeof(endpoint_out_buffer[0]),
	                                                       (int16_t**) endpoint_in_buffer,
	                                                       sizeof(endpoint_in_buffer) / sizeof(endpoint_in_buffer[0]),
	                                                       nframes);

	// Put in usbBuffers IN samples from previous audio processing period
	// Doing this before audio processing ensure more constant buffering as
	// we update usbBuffers always early in the audio processing period, never late even with heavy audio processing.
	for(size_t i = 0; i < AUDIO_IN_NUMBER; i++) {
		size_t writtenSize = usbBuffers[DUB_In + i].writeAudioSample(true, &usb_buffers[DUB_In + i], nframes);
		if(USBD_AUDIO_IsEndpointEnabled(true, i) && writtenSize != nframes) {
			GLITCH_DETECTION_increment_counter(GT_UsbInOverrun);
		}
	}

	AudioProcessor::getInstance()->updateCpuUsage();

	// MPU_Config(1);

	TRACING_add(false, 0, "Audio Processing end");
}

/**
 * @brief Tx Transfer Half completed callback.
 * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval None
 */
extern "C" void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	DAMC_processAudioFromDMAInterrupt();
}
/**
 * @brief Tx Transfer second Half completed callback.
 * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval None
 */
extern "C" void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	DAMC_processAudioFromDMAInterrupt();
}

uint32_t DAMC_getUSBFeedbackValue(enum DAMC_USB_Buffer_e index) {
	return usbBuffers[index].getUSBFeedbackValue();
}

uint32_t DAMC_getUSBInSizeValue(enum DAMC_USB_Buffer_e index) {
	return usbBuffers[index].getUSBInSizeValue();
}

void DAMC_resetBufferProcessedFlags() {
	for(auto& buffer : usbBuffers)
		buffer.resetBufferProcessedFlagFromAudio();
}

/** Reset USB circular buffers
 * The buffer are reset just before starting the USB audio stream and before
 * checking the expected buffering for USB feedback
 * (before DAMC_getUSBFeedbackValue/DAMC_getUSBInSizeValue call).
 *
 * Clear the buffer with the appropriate available silence
 * samples in the buffer so we don't get a buffer overflow or underflow
 * before the feedback to USB Host adjust the buffer margin.
 */
void DAMC_resetAudioBuffer(enum DAMC_USB_Buffer_e index) {
	usbBuffers[index].resetAudioBufferFromUSB();
}

size_t DAMC_writeAudioSampleFromUSB(enum DAMC_USB_Buffer_e index, const void* data, size_t size) {
	return usbBuffers[index].writeAudioSample(false, data, size);
}

size_t DAMC_readAudioSampleFromUSB(enum DAMC_USB_Buffer_e index, void* data, size_t size) {
	return usbBuffers[index].readAudioSample(false, data, size);
}

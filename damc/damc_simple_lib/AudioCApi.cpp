#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "CodecAudio.h"
#include "TimeMeasure.h"
#include "usbd_audio.h"

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

void DAMC_setControlFromUSB(
    uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest, uint16_t value) {
	AudioProcessor::getInstance()->getControls()->setControlFromUSB(
	    unit_id, control_selector, channel, bRequest, value);
}

uint16_t DAMC_getControlFromUSB(uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest) {
	return AudioProcessor::getInstance()->getControls()->getControlFromUSB(
	    unit_id, control_selector, channel, bRequest);
}

/**
 * @brief Tx Transfer Half completed callback.
 * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval None
 */
extern "C" void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
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
		DAMC_readAudioSample((enum DAMC_USB_Buffer_e)(DUB_Out1 + i), &usb_buffers[DUB_Out1 + i], nframes);
	}

	AudioProcessor::getInstance()->processAudioInterleaved((const int16_t**) endpoint_out_buffer,
	                                                       sizeof(endpoint_out_buffer) / sizeof(endpoint_out_buffer[0]),
	                                                       (int16_t**) endpoint_in_buffer,
	                                                       sizeof(endpoint_in_buffer) / sizeof(endpoint_in_buffer[0]),
	                                                       nframes);

	for(size_t i = 0; i < AUDIO_IN_NUMBER; i++) {
		DAMC_writeAudioSample((enum DAMC_USB_Buffer_e)(DUB_In + i), &usb_buffers[DUB_In + i], nframes);
	}
}
/**
 * @brief Tx Transfer Half completed callback.
 * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval None
 */
extern "C" void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	BSP_AUDIO_OUT_HalfTransfer_CallBack();
}

CircularBuffer<3> usbBuffers[3];
int32_t diff_usb[2];
int32_t expected_buff[2];
int32_t usb_buff[2];
int32_t dma_pos_at_usb[2];
int32_t div_ratio = 1;
uint32_t DAMC_getUSBFeedbackValue(enum DAMC_USB_Buffer_e index) {
	uint32_t feedback = 6 << 16;  // nominal value: 6 samples per microframe

	// Expected = 1.25 buffer (= 1.25 * 48 samples) + remaning DMA playback size
	// We are running just before getting a new buffer, so remove 48 from the expected result
	uint32_t dma_pos = CodecAudio::instance.getDMAPos();
	uint32_t expected_buffering = (48 / 4) + (dma_pos % 48);
	uint32_t usb_buffering = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());
	int32_t diff_buffering = usb_buffering - expected_buffering;
	diff_usb[index] = diff_buffering;
	expected_buff[index] = expected_buffering;
	usb_buff[index] = usb_buffering;
	dma_pos_at_usb[index] = dma_pos;

	feedback -= diff_buffering * (65536 / 8192) * div_ratio;
	//	if(diff_buffering > 10) {
	//		// diff_buffering is number of sample diff
	//		// we want to resolve this for the next second (in 8192 microframes)
	//		// As the feedback is the number of sample per microframe (and not per second)
	//		// we need to add diff_buffering/128
	//		feedback -= diff_buffering * (65536 / 8192);
	//		feedback -= (6 << 16) / div_ratio;
	//	} else if(diff_buffering < 10) {
	//		feedback += (6 << 16) / div_ratio;
	//	}
	return feedback;
}

uint32_t available_usb_buffer[3];
void DAMC_writeAudioSample(enum DAMC_USB_Buffer_e index, const void* data, size_t size) {
	usbBuffers[index].writeOutBuffer(usbBuffers[index].getReadPos(), (const uint32_t*) data, size);
	available_usb_buffer[index] = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());
}
size_t DAMC_readAudioSample(enum DAMC_USB_Buffer_e index, void* data, size_t size) {
	return usbBuffers[index].readInBuffer(usbBuffers[index].getWritePos(), (uint32_t*) data, size);
}

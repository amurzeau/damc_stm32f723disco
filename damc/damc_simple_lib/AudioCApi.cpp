#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "CodecAudio.h"
#include "TimeMeasure.h"
#include "main.h"
#include "usbd_audio.h"
#include <uv.h>

void DAMC_init() {
	// This will allocate instance
	AudioProcessor::getInstance();
	TimeMeasure::updateClockPerUs();
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

void DAMC_usbInterruptBeginMeasure() {
	TimeMeasure::timeMeasure[TMI_UsbInterrupt].beginMeasure();
}

void DAMC_usbInterruptEndMeasure() {
	TimeMeasure::timeMeasure[TMI_UsbInterrupt].endMeasure();
}

void DAMC_mainLoopBeginMeasure() {
	TimeMeasure::timeMeasure[TMI_MainLoop].beginMeasure();
}

void DAMC_mainLoopEndMeasure() {
	TimeMeasure::timeMeasure[TMI_MainLoop].endMeasure();
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
	// Ensure PSRAM is not acceeded while in audio interrupt
	//MPU_Config(0);
	TimeMeasure::on1msElapsed();

	TimeMeasure::timeMeasure[TMI_AudioProcessing].beginMeasure();

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

	TimeMeasure::timeMeasure[TMI_AudioProcessing].endMeasure();
	//MPU_Config(1);
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

__attribute__((used)) CircularBuffer<uint32_t, 3, false> usbBuffers[3];
__attribute__((used)) int32_t diff_usb[3];
__attribute__((used)) int32_t expected_buff[3];
__attribute__((used)) int32_t usb_buff[3];
__attribute__((used)) int32_t dma_pos_at_usb[3];
__attribute__((used)) int32_t div_ratio = 1;
uint32_t DAMC_getUSBFeedbackValue(enum DAMC_USB_Buffer_e index) {
	uint32_t feedback = 6 << 16;  // nominal value: 6 samples per microframe

	// Expected = 1.25 buffer (= 1.25 * 48 samples) + remaning DMA playback size
	// We are running just before getting a new buffer, so remove 48 from the expected result
	uint32_t dma_pos = CodecAudio::instance.getDMAOutPos() % 48;
	uint32_t expected_buffering = (48 / 4) + dma_pos;
	uint32_t usb_buffering = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());
	int32_t diff_buffering = usb_buffering - expected_buffering;
	expected_buff[index] = expected_buffering;
	usb_buff[index] = usb_buffering;
	dma_pos_at_usb[index] = dma_pos;

	if(dma_pos < 10) {
		if(diff_buffering < -24) {
			diff_buffering += 48;
		} else if(diff_buffering > 24) {
			diff_buffering -= 48;
		}
	}

	diff_usb[index] = diff_buffering;

	feedback -= diff_buffering * (65536 / 8192) * div_ratio;
	return feedback;
}

__attribute__((used)) int32_t diff_usb_before_adjust[3];
__attribute__((used)) int32_t is_underdma;
uint32_t DAMC_getUSBInSizeValue(enum DAMC_USB_Buffer_e index) {
	uint32_t usb_in_size = 48 << 16;  // nominal value: 48 samples per microframe

	// Expected = 1.25 buffer (= 1.25 * 48 samples) + remaning DMA playback size
	// We are running just before getting a new buffer, so remove 48 from the expected result
	uint32_t dma_pos = CodecAudio::instance.getDMAInPos() % 48;
	uint32_t expected_buffering = 48 + (48 / 4) + 48 - dma_pos;
	uint32_t usb_buffering = usbBuffers[index].getAvailableWriteForDMA(usbBuffers[index].getWritePos());
	int32_t diff_buffering = usb_buffering - expected_buffering;
	diff_usb_before_adjust[index] = diff_buffering;

	if(dma_pos < 10) {
		if(diff_buffering < -24) {
			diff_buffering += 48;
		} else if(diff_buffering > 24) {
			diff_buffering -= 48;
		}
	}

	diff_usb[index] = diff_buffering;
	expected_buff[index] = expected_buffering;
	usb_buff[index] = usb_buffering;
	dma_pos_at_usb[index] = dma_pos;

	usb_in_size += diff_buffering * (65536 / 1024) * div_ratio;
	return usb_in_size;
}

__attribute__((used)) uint32_t available_usb_buffer[3];
void DAMC_writeAudioSample(enum DAMC_USB_Buffer_e index, const void* data, size_t size) {
	usbBuffers[index].writeOutBuffer(usbBuffers[index].getReadPos(), (const uint32_t*) data, size);
	available_usb_buffer[index] = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());
}
size_t DAMC_readAudioSample(enum DAMC_USB_Buffer_e index, void* data, size_t size) {
	return usbBuffers[index].readInBuffer(usbBuffers[index].getWritePos(), (uint32_t*) data, size);
}

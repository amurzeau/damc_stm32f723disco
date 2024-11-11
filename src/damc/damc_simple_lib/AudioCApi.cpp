#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "CodecAudio.h"
#include "GlitchDetection.h"
#include "TimeMeasure.h"
#include "Tracing.h"
#include "usbd_conf.h"
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

__attribute__((used)) CircularBuffer<uint32_t, 3, false> usbBuffers[3];

/**
 * @brief Tx Transfer Half completed callback.
 * @param  hsai pointer to a SAI_HandleTypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval None
 */
extern "C" void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
	TRACING_add(false, 0, "Audio Processing start");
	// Ensure PSRAM is not acceeded while in audio interrupt
	// MPU_Config(0);
	TimeMeasure::on1msElapsed();

	// Reset usbBuffers processed flag
	// Used to adjust expected buffer size with USB
	for(auto& buffer : usbBuffers)
		buffer.resetBufferProcessedFlag();

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
		size_t readSize =
		    DAMC_readAudioSample((enum DAMC_USB_Buffer_e)(DUB_Out1 + i), &usb_buffers[DUB_Out1 + i], nframes);
		if(USBD_AUDIO_IsEndpointEnabled(false, i) && readSize != nframes) {
			GLITCH_DETECTION_increment_counter(GT_UsbOutUnderrun);
		}
	}

	AudioProcessor::getInstance()->processAudioInterleaved((const int16_t**) endpoint_out_buffer,
	                                                       sizeof(endpoint_out_buffer) / sizeof(endpoint_out_buffer[0]),
	                                                       (int16_t**) endpoint_in_buffer,
	                                                       sizeof(endpoint_in_buffer) / sizeof(endpoint_in_buffer[0]),
	                                                       nframes);

	for(size_t i = 0; i < AUDIO_IN_NUMBER; i++) {
		if(USBD_AUDIO_IsEndpointEnabled(true, i)) {
			size_t writtenSize =
			    DAMC_writeAudioSample((enum DAMC_USB_Buffer_e)(DUB_In + i), &usb_buffers[DUB_In + i], nframes);
			if(writtenSize != nframes) {
				GLITCH_DETECTION_increment_counter(GT_UsbInOverrun);
			}
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
extern "C" void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
	BSP_AUDIO_OUT_HalfTransfer_CallBack();
}

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

	// When the DMA is near the end of the codec buffer (dma_pos near 48),
	// this means the DAMC audio processing interrupt (triggered by DMA interrupt)
	// will trigger soon which means we need at least 48 samples already available
	// for processing.
	// When the DMA is at the beginning of the codec buffer (dma_pos near 0)
	// the DAMC audio processing interrupt just executed (or is executing)
	// which means the USB buffer should be almost empty (0.25 buffer for margin).
	// To follow these rules, the buffering need to follow the DMA position.
	//
	// Audio latency of the next sample to be added to USB buffer is:
	//  USB buffering (0.25 buffer margin + dma_pos)
	//  + remaining time before next DAMC audio interrupt
	//  + 1 buffer of DMA delay before the sample is sent to codec (codec double buffering)
	//  + codec I2S to analog latency
	// Which means:
	//  0.25 buffer margin
	//  + 1 buffer of USB/DAMC sync buffering
	//  + 1 buffer of DMA codec double buffering
	//  + codec latency
	//
	//  Period: 1ms == 1 buffer
	//  "1" is the followed sample
	//  Received from USB OUT:  1|   |   |   |   |
	//  DAMC audio processing:   |   1   |   |   |
	//  Copy to codec buffer:    |   | 1 |   |   |
	//  Sent by DMA to codec:    |   |   1   |   |
	//
	// Latency = 2.25ms + codec latency
	uint32_t expected_buffering = (48 / 4) + dma_pos;

	// If the usb buffer is not yet read by DAMC processing interrupt, we expect 1 buffer more
	if(!usbBuffers[index].isBufferProcessed())
		expected_buffering += 48;

	uint32_t usb_buffering = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());
	int32_t diff_buffering = usb_buffering - expected_buffering;
	expected_buff[index] = expected_buffering;
	usb_buff[index] = usb_buffering;
	dma_pos_at_usb[index] = dma_pos;

	// Concurrency cases:
	// dma_pos is at end of buffer:
	//   DMA might have triggered the interrupt, but USB interrupt has higher priority
	//   => usbBuffers won't be processed
	//
	// dma_pos is at beginning of buffer:
	//   - DAMC audio processing didn't have time to reset usbBuffers processing flags
	//     => can happen when dma_pos is still low (assuming < 10 which is around 21µs)
	//
	//   - DAMC audio processing is still at the start of the processing handler and didn't read data from usbBuffers
	//   but flags were reset
	//     => taken into account using isBufferProcessed()
	//
	//   - DAMC audio processing already read data from usbBuffers
	//     => taken into account using isBufferProcessed()
	//
	//   - The USB interrupt is just after having updated the usbBuffers
	//     pointers and before having set the bufferProcessed flag
	//     => this should statisticaly not happen multiple times in a row and should be handled by dma_pos < 10

	// If dma_pos is low, we might have concurrency issues, so assume
	// diff_buffering of more than 24 needs +/-48 samples adjustement
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
	uint32_t usb_in_size = 48 << 16;  // nominal value: 48 samples per transfer

	// Expected = 1.25 buffer (= 1.25 * 48 samples) + remaning DMA playback size
	uint32_t dma_pos = CodecAudio::instance.getDMAInPos() % 48;

	// When the DMA is near the end of the codec buffer (dma_pos near 48),
	// this means the DAMC audio processing interrupt (triggered by DMA interrupt)
	// will trigger soon which means we need at least 48 samples of available USB buffer
	// for processing.
	// When the DMA is at the beginning of the codec buffer (dma_pos near 0)
	// the DAMC audio processing interrupt just executed (or is executing)
	// which means the USB buffer should be almost empty (0.25 buffer for margin).
	// If the DAMC audio processing is still executing, we might still have 1.25 buffer available.
	// To follow these rules, the buffering need to follow the DMA position.
	//
	// We are running just before a buffer is read from USB buffer,
	// so add a full buffer of 48 samples to the expected buffering.
	// If DAMC audio processing interrupt will execute soon (high dma_pos near 48), we need the smallest usb bufferring
	// of 48 samples (that are going to be read just afer this function) + 0.25 buffer of margin.
	//
	// Audio latency of a sample to be read by codec ADC:
	//  Codec analog to I2S latency
	//  + 1 buffer of DMA delay before the sample is read by DAMC audio interrupt (codec double buffering)
	//  + 1 buffer of USB/DAMC sync buffering
	//
	//  Period: 1ms == 1 buffer
	//  "1" is the followed sample
	//
	//  DMA write to codec buffer: 1   |   |   |
	//  DAMC audio processing:     |   1   |   |
	//  Copy to USB buffer:        |   | 1 |   |
	//  Sent to USB IN endpoint:   |   |   |1  |
	// Latency = 2.25ms + codec latency
	uint32_t expected_buffering = 48 + (48 / 4) + (48 - dma_pos);

	// If the usbBuffers is not yet written by DAMC processing interrupt, we expect 1 buffer less
	if(!usbBuffers[index].isBufferProcessed())
		expected_buffering -= 48;

	uint32_t usb_buffering = usbBuffers[index].getAvailableWriteForDMA(usbBuffers[index].getWritePos());
	int32_t diff_buffering = usb_buffering - expected_buffering;
	diff_usb_before_adjust[index] = diff_buffering;

	// Concurrency cases:
	// dma_pos is at end of buffer:
	//   DMA might have triggered the interrupt, but USB interrupt has higher priority
	//   => usbBuffers won't be processed
	//
	// dma_pos is at beginning of buffer:
	//   - DAMC audio processing didn't have time to reset usbBuffers processing flags
	//     => can happen when dma_pos is still low (assuming < 10 which is around 21µs)
	//
	//   - DAMC audio processing is still at the start of the processing handler and didn't write data to usbBuffers
	//   but flags were reset
	//     => taken into account using isBufferProcessed()
	//
	//   - DAMC audio processing already written data to usbBuffers
	//     => taken into account using isBufferProcessed()
	//
	//   - The USB interrupt is just after having updated the usbBuffers
	//     pointers and before having set the bufferProcessed flag
	//     => this should statisticaly not happen multiple times in a row but will lead to bad diff_buffering value

	// If dma_pos is low, we might have not have correctly reset the processing flags, so assume
	// diff_buffering of more than 24 needs +/-48 samples adjustement
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

void DAMC_resetAudioBuffer(enum DAMC_USB_Buffer_e index) {
	usbBuffers[index].reset();
}

__attribute__((used)) uint32_t available_usb_buffer[3];
size_t DAMC_writeAudioSample(enum DAMC_USB_Buffer_e index, const void* data, size_t size) {
	size_t writtenSize = usbBuffers[index].writeOutBuffer(usbBuffers[index].getReadPos(), (const uint32_t*) data, size);

	available_usb_buffer[index] = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());

	return writtenSize;
}

size_t DAMC_readAudioSample(enum DAMC_USB_Buffer_e index, void* data, size_t size) {
	return usbBuffers[index].readInBuffer(usbBuffers[index].getWritePos(), (uint32_t*) data, size);
}

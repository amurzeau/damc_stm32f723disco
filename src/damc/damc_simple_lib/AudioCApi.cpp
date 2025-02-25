#include "AudioCApi.h"
#include "AudioProcessor.h"
#include "CodecAudio.h"
#include "GlitchDetection.h"
#include "TimeMeasure.h"
#include "Tracing.h"
#include "usbd_conf.h"
#include <usbd_audio.h>
#include <uv.h>

__attribute__((used)) CircularBuffer<uint32_t, 3, false> usbBuffers[3];

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

void DAMC_resetBufferProcessedFlags() {
	// Reset usbBuffers processed flag
	// Used to adjust expected buffer size with USB
	for(auto& buffer : usbBuffers)
		buffer.resetBufferProcessedFlag();
}

__attribute__((used)) int32_t diff_usb[3];
__attribute__((used)) int32_t expected_buff[3];
__attribute__((used)) int32_t usb_buff[3];
__attribute__((used)) int32_t dma_pos_at_usb[3];
__attribute__((used)) int32_t div_ratio = 1;

__attribute__((used)) int32_t feedback_glitch_buffer_processed[3];
__attribute__((used)) int32_t feedback_glitch_dma_pos[3];
__attribute__((used)) int32_t feedback_glitch_dma_interrupt_flag[3];
__attribute__((used)) int32_t feedback_glitch_usb_buffering[3];
__attribute__((used)) int32_t feedback_glitch_diff[3];
__attribute__((used)) uint32_t hitcount = 0;
__attribute__((used)) int32_t dummy_glitch;

/** Returns the usbBuffers available samples assuming it is read when DMA pos reset 0 atomically.
 * This function check if the DMA pos reseted and usbBuffers is not yet processed to adjust the usbBuffers available
 * sample accordingly.
 *
 * Low audio processing CPU usage:
 * usbBuffers available data
 * 60     / |                        / |                       /
 *          |                    /     |                    /
 *          |                /         |                /
 *          |            /             |            /
 *          |        /                 |        /
 *          |    /                     |    /
 * 12       |/                         |/
 * DMAOutPos
 *  48                            / |                        / |
 *                            /     |                    /     |
 *                        /         |                /         |
 *                    /             |            /             |
 *                /                 |        /                 |
 *            /                     |    /                     |
 *   0   _/                         |/                         |
 *       |  R                       |  R                       |
 *
 *
 *
 * High audio processing CPU usage, for a buffer lately processed at 99% of 1ms period
 * usbBuffers available data
 * 108                           / |                        / |
 *                           /     |                    /     |
 *                       /         |                /         |
 * 84                /             |            /             |
 *               /                 |        /                 |
 *           /                     |    /                     |
 * 60    /                         |/                         |/
 * DMAOutPos
 *  48                            / |                        / |
 *                            /     |                    /     |
 *                        /         |                /         |
 *                    /             |            /             |
 *                /                 |        /                 |
 *            /                     |    /                     |
 *   0   _/                         |/                         |
 *       |                         R|                         R|
 *
 * One processing late:
 * usbBuffers available data (max = 144)
 * 116                                                           /|
 *                                                           /    |
 *                                                       /        |
 *                                                   /            |
 *                                               /                |
 * 84                                        /                    |    /|
 * 76                                    /                        |/    |
 * 68      /|                        /                                  |                   /|                      /|
 * 60   /   |                    /                                      |                /   |                  /
 *          |                /                                          |            /       |              /
 *          |            /                                              |        /           |          /
 *          |        /                                                  |    /               |      /
 * 28       |    /                                                      |/                   |  /
 * 20       |/                                                                               /
 * DMAOutPos
 *  48                            / |                        / |                        / |                        / |
 *                            /     |                    /     |                    /     |                    /     |
 *                        /         |                /         |                /         |                /         |
 *                    /             |            /             |            /             |            /             |
 *                /                 |        /                 |        /                 |        /                 |
 *            /                     |    /                     |    /                     |    /                     |
 *   0   _/                         |/                         |/                         |/                         |
 *       |  R                       |                          |  R    R                  |  R                       |
 *
 * usbBuffers converted in DMAOutPos domain + 16 (margin)
 * 84                                                                 /|
 * 76                                                              /   |
 * 68                                                           /      |
 * 60    |                        / |                        /         |                / |                        / |
 *       |                    /     |                    /             |            /     |                    /     |
 *       |                /         |                /                 |        /         |                /         |
 *       |            /             |            /                     |    /             |            /             |
 * 28    |        /                 |        /                         |/                 |        /                 |
 * 20    |    /                     |    /                                                |    /                     |
 * 16    |/                         |/                                                    |/                         |
 * adj:  ---+                       ------------------------------+    *                  ---+                       -
 */
static int32_t DAMC_adjustUsbBufferAvailableForCurrentProcessingPeriod(enum DAMC_USB_Buffer_e index,
                                                                       int32_t usb_buffering) {
	// Note: DMA ISR is cleared after usbBuffers.resetBufferProcessedFlag is called
	// in AUDIO_OUT_SAIx_DMAx_IRQHandler().
	// This function is called from USB interrupt, so DMA ISR can't preempt this function.
	// This means we need to check the buffer processed flag before, then the DMA ISR flag
	// To ensure that when reading usbBuffers flag and available samples

	// Concurrency cases:
	// dma_pos is at end of buffer:
	//   DMA might have triggered the interrupt, but USB interrupt has higher priority
	//   => usbBuffers won't be processed, DMA ISR and usbBuffers processed flag will be both set
	//
	// dma_pos is at beginning of buffer:
	//   - DAMC audio processing didn't have time to reset usbBuffers processing flags
	//     => usbBuffers won't be processed, DMA ISR and usbBuffers processed flag will be both set
	//
	//   - DAMC audio processing is still at the start of the processing handler and didn't read data from usbBuffers
	//   but flags were reset
	//     => taken into account using isBufferProcessed()
	//     => usbBuffers won't be processed, either DMA ISR is still set or usbBuffers processed flag is cleared
	//
	//   - DAMC audio processing already read data from usbBuffers
	//     => usbBuffers is processed, DMA ISR is cleared and usbBuffers processed flag is set
	//
	//   - The USB interrupt is just after having updated the usbBuffers
	//     pointers and before having set the bufferProcessed flag
	//     => not possible, pointers and flag are updated in a single 32 bits write in CircularBuffer.h

	bool buffer_processed;
	if(index != DUB_In)
		buffer_processed = usbBuffers[index].isBufferRead();
	else
		buffer_processed = usbBuffers[index].isBufferWritten();

	// Always use DMAOutPos even for IN endpoint as the audio processing period is defined by the DMA Out interrupts.
	// DMA In is in sync with DMA Out and is a the same position.
	uint32_t dma_pos = CodecAudio::instance.getDMAOutPos() % 48;

	// When dma_pos == 0, we need to do a dummy SAI peripheral read to ensure the
	// DMA ISR interrupt flag is set even if the DMA transfer was still in fligh when we read dma_pos.
	// Doing the dummy read takes around 0.5µs max (4 APB cycles at 13.5 Mhz + cost of instructions).
	// This makes the dummy read almost cost-free compared to more complex code to workaround this case.
	//
	// The DMA does this to process a transfer Peripheral -> Memory:
	//  - The peripheral requests a transfer
	//  - DMA arbitrer wait for AHB/APB to be ready (wait if another transfer is being processed by AHB/APB)
	//  - Decrement NDTR
	//  - Do the peripheral register read
	//  - Do the memory write
	//  - Set HTIF or TCIF interrupt flag if NDTR == half or 0
	//
	// When doing Memory -> Peripheral:
	//  - The peripheral requests a transfer
	//  - DMA arbitrer wait for AHB/APB to be ready (wait if another transfer is being processed by AHB/APB)
	//  - Do the memory write
	//  - Decrement NDTR
	//  - Do the peripheral register read
	//  - Set HTIF or TCIF interrupt flag if NDTR == half or 0
	// https://www.eevblog.com/forum/microcontrollers/32f4-3-ways-to-detect-end-of-dma-transfer/
	// http://efton.sk/STM32/gotcha/g20.html
	bool dma_interrupt_flag = CodecAudio::instance.isAudioProcessingInterruptPending(dma_pos == 0);

	int32_t saved_buffer_processed = buffer_processed;
	int32_t saved_dma_pos = dma_pos;
	int32_t saved_dma_interrupt_flag = dma_interrupt_flag;
	int32_t saved_usb_buffering = usb_buffering;

	// Handle DMA pos and flag mismatch
	if(dma_interrupt_flag && dma_pos > 48 / 2) {
		// We read dma_pos before the DMA pos reset and
		// the DMA ISR flag after the DMA pos reset (which triggered the ISR)
		// Assume the flag is not set to match dma_pos.
		dma_interrupt_flag = false;
	}

	// usbBuffers pos and processed flag mismatch can't happen as they are updated at the same time
	// using ".raw = Offset{true, end}.raw" in writeOutBuffer/readInBuffer as a 32 bits write.

	/*
	 * Check if we are in the window between DMA pos reset and usbBuffers processed
	 * Events order:
	 *
	 * Normal ISR handling without latency (exection time not scaled):
	 * DMA pos reset:               |                 |
	 * DMA ISR flag:               _-------___________-------___________
	 * DMA ISR execution:          ____-----------_______-----------____
	 * usbBuffers processed flag:  _------___---------------___---------
	 * usbBuffers need adjustment: _---------_________---------_________
	 *
	 *
	 * ISR handling with late ISR and DMA pos reset while ISR execution
	 * DMA pos reset:               |                 |                 |
	 * DMA ISR flag:               _-------___________---------------___---------___________
	 * DMA ISR execution:          ____-----------_______________-----------_-----------____
	 * usbBuffers processed flag:  _------___-----------------------___---------___---------
	 * usbBuffers need adjustment: _---------_________-----------------_-----------_________
	 *
	 * "usbBuffers need adjustment" truth table:
	 *
	 *
	 * | usbBuffers need adjustment | DMA ISR flag | usbBuffers processed flag |
	 * | -------------------------- | ------------ | ------------------------- |
	 * | true                       | false        | false                     |
	 * | false                      | false        | true                      |
	 * | true                       | true         | false                     |
	 * | true                       | true         | true                      |
	 *
	 * Condition: (DMA ISR flag) || !(usbBuffers processed flag)
	 */
	if(dma_interrupt_flag || !buffer_processed) {
		// We expect usbBuffers to be read in the current audio processing period
		// Adjust the amount of samples as if the DMA ISR and buffer processing were done at the same time atomically.
		if(index != DUB_In) {
			usb_buffering -= 48;
		} else {
			usb_buffering += 48;
		}
	}

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

	int32_t expected_buffering;

	if(index != DUB_In) {
		// Expected = 1.25 buffer (= 1.25 * 48 samples) + elapsed time since previous DMA pos reset
		// We are running just before getting a new buffer, so remove 48 from the expected result
		expected_buffering = (48 / 4) + dma_pos;
	} else {
		// Expected = 1.25 buffer (= 1.25 * 48 samples) + remaining DMA record size
		expected_buffering = 48 + (48 / 4) + (48 - dma_pos);
	}

	// Update debug monitoring varialbes
	expected_buff[index] = expected_buffering;
	dma_pos_at_usb[index] = dma_pos;

	if(usb_buffering != 0 && (usb_buffering - expected_buffering > 24 || usb_buffering - expected_buffering < -24)) {
		feedback_glitch_buffer_processed[index] = saved_buffer_processed;
		feedback_glitch_dma_pos[index] = saved_dma_pos;
		feedback_glitch_dma_interrupt_flag[index] = saved_dma_interrupt_flag;
		feedback_glitch_usb_buffering[index] = saved_usb_buffering;
		feedback_glitch_diff[index] = usb_buffering - expected_buffering;
	}

	hitcount++;

	return usb_buffering - expected_buffering;
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
	uint32_t feedback = 6 << 16;  // nominal value: 6 samples per microframe

	uint32_t usb_buffering = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());
	int32_t diff_buffering = DAMC_adjustUsbBufferAvailableForCurrentProcessingPeriod(index, usb_buffering);
	usb_buff[index] = usb_buffering;
	diff_usb[index] = diff_buffering;

	// Assuming 0 clock drift, when 1 sample is missing, the host will compensate it in
	// 8192 microframes, that is 1.024 second.
	// 8192 is a convient value leading to integer operations.
	// 65536 is the ratio for the decimal part of feedback, being in 16.16 fixed point format.
	// So 1.0f == 65536.
	feedback -= diff_buffering * (65536 / 8192) * div_ratio;
	return feedback;
}

__attribute__((used)) int32_t is_underdma;
uint32_t DAMC_getUSBInSizeValue(enum DAMC_USB_Buffer_e index) {
	uint32_t usb_in_size = 48 << 16;  // nominal value: 48 samples per transfer

	uint32_t usb_buffering = usbBuffers[index].getAvailableWriteForDMA(usbBuffers[index].getWritePos());
	int32_t diff_buffering = DAMC_adjustUsbBufferAvailableForCurrentProcessingPeriod(index, usb_buffering);

	usb_buff[index] = usb_buffering;
	diff_usb[index] = diff_buffering;

	usb_in_size += diff_buffering * (65536 / 1024) * div_ratio;
	return usb_in_size;
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
	// Get expected buffering
	int32_t expected_buffering = -DAMC_adjustUsbBufferAvailableForCurrentProcessingPeriod(index, 0);

	usbBuffers[index].reset(expected_buffering);
}

__attribute__((used)) uint32_t available_usb_buffer[3];
size_t DAMC_writeAudioSample(enum DAMC_USB_Buffer_e index, const void* data, size_t size) {
	size_t writtenSize = usbBuffers[index].writeOutBuffer(usbBuffers[index].getReadPos(), (const uint32_t*) data, size);

	available_usb_buffer[index] = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());

	return writtenSize;
}

__attribute__((used)) size_t available_usb_buffer_read[3];
__attribute__((used)) size_t available_usb_buffer_read_underflow[3];
size_t DAMC_readAudioSample(enum DAMC_USB_Buffer_e index, void* data, size_t size) {
	available_usb_buffer_read[index] = usbBuffers[index].getAvailableReadForDMA(usbBuffers[index].getReadPos());

	size_t readSize = usbBuffers[index].readInBuffer(usbBuffers[index].getWritePos(), (uint32_t*) data, size);

	if(readSize != size) {
		available_usb_buffer_read_underflow[index] = available_usb_buffer_read[index];
	}

	return readSize;
}

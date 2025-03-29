#include "USBAudioBuffers.h"
#include "CodecAudio.h"
#include "Tracing.h"
#include <atomic>
#include <stdint.h>

UsbAudioBuffer usbBuffers[3] = {
    {DUB_Out1, UsbAudioBuffer::DirectionUsbToAudio},
    {DUB_Out2, UsbAudioBuffer::DirectionUsbToAudio},
    {DUB_In, UsbAudioBuffer::DirectionAudioToUsb},
};

void UsbAudioBuffer::resetBufferProcessedFlagFromAudio() {
	// Reset usbBuffers processed flag at the beginning for the audio processing ISR
	// Used to adjust expected buffer size with USB
	buffer.resetBufferProcessedFlag();
}

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
int32_t UsbAudioBuffer::adjustUsbBufferAvailableForCurrentProcessingPeriodFromUSB(bool inReset, int32_t usb_buffering) {
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
	if(direction == DirectionUsbToAudio)
		buffer_processed = buffer.isBufferRead();
	else
		buffer_processed = buffer.isBufferWritten();

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
		// Adjust only for theses cases:
		// - a DAMC_readAudioSample/DAMC_writeAudioSample will occur later in the audio processing period and will
		// transfer 48 samples
		// - a DAMC_readAudioSample/DAMC_writeAudioSample is in progress which could lead to a short read/write if the
		// buffer is empty or full.

		// Reseting while the buffer is full/empty might be ignored

		if(processingByAudioISR.load(std::memory_order_relaxed) == OwnerUsedByAudioProcess) {
			data_after_reset++;

			// We interrupted a DAMC_readAudioSample, reseting won't have any effect and only the current available
			// data will be read instead of 48 samples.
			uint32_t available_data = usb_buffering;
			if(available_data > 48)
				available_data = 48;
			available_data_for_processing = available_data;
			usb_buffering -= available_data;
		} else {
			// The buffer will be processed later in the audio period.
			// If we reset before the start of reading/writing the buffer by the audio processing ISR, it will
			// read/write a full 48 samples as we will add them just after feedback/reset.
			// Adjust the amount of samples as if the DMA ISR and buffer processing were done at the same time
			// atomically.
			usb_buffering -= 48;
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
	// For input:
	//  Received from codec with DMA:  1   |   |   |   |
	//  DAMC audio processing:         |   | 1 |   |   |
	//  Copy to usbBuffer:             |   |  1|   |   |
	//  Received from USB IN:          |   | * |   |1  |
	//
	// Latency = 2.25ms + codec latency

	int32_t expected_buffering;

	// Expected = 1.25 buffer (= 1.25 * 48 samples) + elapsed time since previous DMA pos reset
	// We are running just before getting a new buffer, so remove 48 from the expected result
	expected_buffering = (48 / 4) + dma_pos;

	// Update debug monitoring variables
	expected_buff = expected_buffering;
	dma_pos_at_usb = dma_pos;
	adj_usb_buff = usb_buffering;
	buffer_processed_flag = buffer_processed;
	dma_isr_flag = dma_interrupt_flag;

	if(!inReset && ((usb_buffering - expected_buffering) > 24 || (usb_buffering - expected_buffering) < -24)) {
		feedback_glitch_buffer_processed = saved_buffer_processed;
		feedback_glitch_dma_pos = saved_dma_pos;
		feedback_glitch_dma_interrupt_flag = saved_dma_interrupt_flag;
		feedback_glitch_usb_buffering = saved_usb_buffering;
		feedback_glitch_diff = usb_buffering - expected_buffering;
	}

	return usb_buffering - expected_buffering;
}

uint32_t UsbAudioBuffer::getUSBFeedbackValue() {
	uint32_t feedback = 6 << 16;  // nominal value: 6 samples per microframe

	uint32_t usb_buffering = buffer.getAvailableReadForDMA(buffer.getReadPos());
	int32_t diff_buffering = adjustUsbBufferAvailableForCurrentProcessingPeriodFromUSB(false, usb_buffering);
	usb_buff = usb_buffering;
	diff_usb = diff_buffering;

	TRACING_add_feedback(index,
	                     diff_usb,
	                     expected_buff,
	                     adj_usb_buff,
	                     usb_buff,
	                     dma_pos_at_usb,
	                     buffer_processed_flag,
	                     dma_isr_flag,
	                     available_data_for_processing,
	                     processingByAudioISR,
	                     "Feedback OUT");

	// Assuming 0 clock drift, when 1 sample is missing, the host will compensate it in
	// 8192 microframes, that is 1.024 second.
	// 8192 is a convient value leading to integer operations.
	// 65536 is the ratio for the decimal part of feedback, being in 16.16 fixed point format.
	// So 1.0f == 65536.
	feedback -= diff_buffering * (65536 / 8192);
	return feedback;
}

uint32_t UsbAudioBuffer::getUSBInSizeValue() {
	uint32_t usb_in_size = 48 << 16;  // nominal value: 48 samples per transfer

	uint32_t usb_buffering = buffer.getAvailableWriteForDMA(buffer.getWritePos());
	int32_t diff_buffering = adjustUsbBufferAvailableForCurrentProcessingPeriodFromUSB(false, usb_buffering);

	usb_buff = usb_buffering;
	diff_usb = diff_buffering;

	TRACING_add_feedback(index,
	                     diff_usb,
	                     expected_buff,
	                     adj_usb_buff,
	                     usb_buff,
	                     dma_pos_at_usb,
	                     buffer_processed_flag,
	                     dma_isr_flag,
	                     available_data_for_processing,
	                     processingByAudioISR,
	                     "Feedback IN");

	usb_in_size -= diff_buffering * (65536 / 1024);
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
volatile uint32_t dummy;
void UsbAudioBuffer::resetAudioBufferFromUSB() {
	// Get expected buffering
	uint32_t usb_buffering;

	if(direction == DirectionUsbToAudio) {
		usb_buffering = buffer.getAvailableReadForDMA(buffer.getReadPos());
	} else {
		usb_buffering = buffer.getAvailableWriteForDMA(buffer.getWritePos());
	}

	int32_t expected_buffering = -adjustUsbBufferAvailableForCurrentProcessingPeriodFromUSB(true, usb_buffering);

	expected_buffering += usb_buffering;

	if(processingByAudioISR.load(std::memory_order_relaxed) == OwnerUsedByAudioProcess) {
		dummy = 1;
	}

	if(direction == DirectionUsbToAudio) {
		buffer.resetWritePos(expected_buffering);
	} else {
		buffer.resetReadPos(buffer.getCount() - expected_buffering);
	}
	// Notify audio processing thread that we modified the usb buffer
	processingByAudioISR.store(OwnerPointerChanged, std::memory_order_relaxed);

	TRACING_reset();
	TRACING_add_feedback(index,
	                     0,
	                     expected_buff,
	                     adj_usb_buff,
	                     expected_buffering,
	                     dma_pos_at_usb,
	                     buffer_processed_flag,
	                     dma_isr_flag,
	                     available_data_for_processing,
	                     processingByAudioISR,
	                     "Reset usbBuffers");
}

size_t UsbAudioBuffer::writeAudioSample(bool fromAudioISR, const void* data, size_t size) {
	uint32_t usb_read_pos;
	BufferOwner expected_value;

	if(fromAudioISR) {
		// Ensure atomic read of usb pos with processingByAudioISR.
		// If USB interrupt occurs in-between and cause a usbBuffers reset, retry to read the position.
		// If USB interrupt occurs after having set processingByAudioISR, it will assume we won't see the reset.
		do {
			expected_value = OwnerUnused;
			processingByAudioISR.store(OwnerUnused, std::memory_order_relaxed);

			// Ensure buffer.getReadPos is read while processingByAudioISR is set to OwnerUnused
			// So compare_exchange_strong will fail if an interrupt happens while doing buffer.getReadPos().
			std::atomic_signal_fence(std::memory_order_seq_cst);

			usb_read_pos = buffer.getReadPos();
		} while(!processingByAudioISR.compare_exchange_strong(
		    expected_value, OwnerUsedByAudioProcess, std::memory_order_acquire, std::memory_order_relaxed));
	} else {
		usb_read_pos = buffer.getReadPos();
	}

	size_t writtenSize = buffer.writeOutBuffer(usb_read_pos, (const uint32_t*) data, size);

	if(fromAudioISR) {
		processingByAudioISR.store(OwnerUnused, std::memory_order_release);
	}

	uint32_t available_size = buffer.getAvailableReadForDMA(usb_read_pos);
	uint32_t dma_pos = CodecAudio::instance.getDMAOutPos() % 48;
	TRACING_add_buffer(index, available_size, dma_pos, "Write usbBuffers");

	return writtenSize;
}

size_t UsbAudioBuffer::readAudioSample(bool fromAudioISR, void* data, size_t size) {
	uint32_t usb_write_pos;
	BufferOwner expected_value;

	if(fromAudioISR) {
		// Ensure atomic read of usb pos with processingByAudioISR.
		// If USB interrupt occurs in-between and cause a usbBuffers reset, retry to read the position.
		// If USB interrupt occurs after having set processingByAudioISR, it will assume we won't see the reset.
		do {
			expected_value = OwnerUnused;
			processingByAudioISR.store(OwnerUnused, std::memory_order_relaxed);

			// Ensure buffer.getWritePos is read while processingByAudioISR is set to OwnerUnused
			// So compare_exchange_strong will fail if an interrupt happens while doing buffer.getWritePos().
			std::atomic_signal_fence(std::memory_order_seq_cst);

			usb_write_pos = buffer.getWritePos();
		} while(!processingByAudioISR.compare_exchange_strong(
		    expected_value, OwnerUsedByAudioProcess, std::memory_order_acquire, std::memory_order_relaxed));
	} else {
		usb_write_pos = buffer.getWritePos();
	}

	size_t readSize = buffer.readInBuffer(usb_write_pos, (uint32_t*) data, size);

	if(fromAudioISR) {
		processingByAudioISR.store(OwnerUnused, std::memory_order_relaxed);
	}

	uint32_t available_size = buffer.getAvailableReadForDMA(buffer.getReadPos());
	uint32_t dma_pos = CodecAudio::instance.getDMAOutPos() % 48;
	TRACING_add_buffer(index, available_size, dma_pos, "Read usbBuffers");

	return readSize;
}

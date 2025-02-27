#pragma once

#include "AudioCApi.h"
#include "CircularBuffer.h"
#include <atomic>
#include <stdint.h>
#include <stdlib.h>

// Concurrently accessed by USB ISR and Audio processing ISR
class UsbAudioBuffer {
public:
	enum BufferOwner : uint32_t {
		OwnerUnused,
		OwnerPointerChanged,
		OwnerUsedByAudioProcess,
	};

	enum BufferDirection {
		DirectionUsbToAudio,
		DirectionAudioToUsb,
	};

	UsbAudioBuffer(enum DAMC_USB_Buffer_e index, BufferDirection direction) : index(index), direction(direction) {}

	void resetBufferProcessedFlagFromAudio();

	uint32_t getUSBFeedbackValue();
	uint32_t getUSBInSizeValue();
	void resetAudioBufferFromUSB();
	size_t writeAudioSample(const void* data, size_t size);
	size_t readAudioSample(void* data, size_t size);

protected:
	int32_t adjustUsbBufferAvailableForCurrentProcessingPeriodFromUSB(int32_t usb_buffering);

private:
	CircularBuffer<uint32_t, 3, false> buffer;
	std::atomic<BufferOwner> readOwner = OwnerUnused;
	std::atomic<BufferOwner> writeOwner = OwnerUnused;
	enum DAMC_USB_Buffer_e index;
	BufferDirection direction;

	// Debug
	int32_t diff_usb;
	int32_t expected_buff;
	int32_t adj_usb_buff;
	int32_t usb_buff;
	int32_t dma_pos_at_usb;
	int32_t buffer_processed_flag;
	int32_t dma_isr_flag;
	int32_t available_data_for_processing;
	int32_t data_after_reset;

	int32_t feedback_glitch_buffer_processed;
	int32_t feedback_glitch_dma_pos;
	int32_t feedback_glitch_dma_interrupt_flag;
	int32_t feedback_glitch_usb_buffering;
	int32_t feedback_glitch_diff;
};

extern UsbAudioBuffer usbBuffers[3];

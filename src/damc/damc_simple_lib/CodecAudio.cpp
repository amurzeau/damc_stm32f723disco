#include "CodecAudio.h"
#include "GlitchDetection.h"
#include <spdlog/spdlog.h>

#include <assert.h>
#include <math.h>
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_audio.h>
#include <string.h>

CodecAudio CodecAudio::instance;

CodecAudio::CodecAudio() : useTlvAsMclkMaster(false), previousAvailableDmaIn(0), previousAvailableDmaOut(0) {}

void CodecAudio::start() {
	codecDamcHATInit.init_i2c();
	useTlvAsMclkMaster = codecDamcHATInit.isAvailable();

	if(useTlvAsMclkMaster) {
		codecDamcHATInit.init();
		codecDamcHATInit.startRxDMA(codecBuffers.in_buffer.getBuffer(), codecBuffers.in_buffer.getCount());
		codecDamcHATInit.startTxDMA(codecBuffers.out_buffer.getBuffer(), codecBuffers.out_buffer.getCount());
	} else {
		codecSTM32F723EDiscoInit.init(useTlvAsMclkMaster);

		codecSTM32F723EDiscoInit.init_after_clock_enabled();

		codecSTM32F723EDiscoInit.startRxDMA(codecBuffers.in_buffer.getBuffer(), codecBuffers.in_buffer.getCount());
		codecSTM32F723EDiscoInit.startTxDMA(codecBuffers.out_buffer.getBuffer(), codecBuffers.out_buffer.getCount());
	}
}

volatile uint32_t diff_dma_out;
void CodecAudio::processAudioInterleavedOutput(const CodecFrame* data_input, size_t nframes) {
	uint16_t dma_read_offset = getDMAOutPos();
	uint32_t availableForDma = codecBuffers.out_buffer.getAvailableReadForDMA(dma_read_offset);
	diff_dma_out = availableForDma;

	size_t writtenSize = codecBuffers.out_buffer.writeOutBuffer(dma_read_offset, data_input, nframes);

	// Check Overrun
	if(writtenSize != nframes) {
		GLITCH_DETECTION_increment_counter(GT_CodecOutXRun);
	}

	// Check DMA Underrun
	if(previousAvailableDmaOut < nframes && availableForDma > (nframes + nframes / 2)) {
		GLITCH_DETECTION_increment_counter(GT_CodecOutDmaUnderrun);
	}
	previousAvailableDmaOut = availableForDma;
}

volatile uint32_t diff_dma_in;
void CodecAudio::processAudioInterleavedInput(CodecFrame* data_output, size_t nframes) {
	uint16_t dma_write_offset = getDMAInPos();
	uint32_t availableForDma = codecBuffers.out_buffer.getAvailableWriteForDMA(dma_write_offset);
	diff_dma_in = availableForDma;
	size_t readSize = codecBuffers.in_buffer.readInBuffer(dma_write_offset, data_output, nframes);

	// Check Underrun
	if(readSize != nframes) {
		GLITCH_DETECTION_increment_counter(GT_CodecInXRun);
	}

	// Check DMA Overrun
	if(previousAvailableDmaIn < nframes && availableForDma > (nframes + nframes / 2)) {
		GLITCH_DETECTION_increment_counter(GT_CodecInDmaOverrun);
	}
	previousAvailableDmaIn = availableForDma;
}

uint32_t CodecAudio::getDMAOutPos() {
	uint32_t dma_pos;

	if(useTlvAsMclkMaster) {
		dma_pos = codecDamcHATInit.getTxRemainingCount();
	} else {
		dma_pos = codecSTM32F723EDiscoInit.getTxRemainingCount();
	}

	uint16_t dma_read_offset = codecBuffers.out_buffer.getCount() - ((dma_pos + 1) / 2);
	return dma_read_offset;
}

uint32_t CodecAudio::getDMAInPos() {
	uint32_t dma_pos;

	if(useTlvAsMclkMaster) {
		dma_pos = codecDamcHATInit.getRxRemainingCount();
	} else {
		dma_pos = codecSTM32F723EDiscoInit.getRxRemainingCount();
	}

	uint16_t dma_write_offset = codecBuffers.in_buffer.getCount() - ((dma_pos + 1) / 2);
	return dma_write_offset;
}

bool CodecAudio::isAudioProcessingInterruptPending(bool insertWaitStates) {
	if(useTlvAsMclkMaster) {
		return codecDamcHATInit.isDMAIsrFlagSet(insertWaitStates);
	} else {
		return codecSTM32F723EDiscoInit.isDMAIsrFlagSet(insertWaitStates);
	}
}

void CodecAudio::setMicBias(bool enable) {
	if(useTlvAsMclkMaster) {
		codecDamcHATInit.setMicBias(enable);
	}
}
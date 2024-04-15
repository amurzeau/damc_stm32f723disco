#include "CodecAudio.h"
#include <spdlog/spdlog.h>

#include <assert.h>
#include <math.h>
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_audio.h>
#include <string.h>

CodecAudio CodecAudio::instance;

CodecAudio::CodecAudio() : useTlvAsMclkMaster(false) {}

void CodecAudio::start() {
	codecDamcHATInit.init_i2c();
	useTlvAsMclkMaster = codecDamcHATInit.isAvailable();

	if(useTlvAsMclkMaster) {
		codecDamcHATInit.init();
		codecDamcHATInit.startTxDMA(codecBuffers.out_buffer.getBuffer(), codecBuffers.out_buffer.getSize());
		codecDamcHATInit.startRxDMA(codecBuffers.in_buffer.getBuffer(), codecBuffers.in_buffer.getSize());
	} else {
		codecSTM32F723EDiscoInit.init(useTlvAsMclkMaster);

		codecSTM32F723EDiscoInit.init_after_clock_enabled();

		codecSTM32F723EDiscoInit.startTxDMA(codecBuffers.out_buffer.getBuffer(), codecBuffers.out_buffer.getSize());
		codecSTM32F723EDiscoInit.startRxDMA(codecBuffers.in_buffer.getBuffer(), codecBuffers.in_buffer.getSize());
	}
}

volatile uint32_t diff_dma_out;
void CodecAudio::processAudioInterleavedOutput(const int16_t* data_input, size_t nframes) {
	uint16_t dma_read_offset = getDMAOutPos();
	diff_dma_out = codecBuffers.out_buffer.getAvailableReadForDMA(dma_read_offset);
	codecBuffers.out_buffer.writeOutBuffer(dma_read_offset, (CodecFrame*) data_input, nframes);
}

volatile uint32_t diff_dma_in;
void CodecAudio::processAudioInterleavedInput(int16_t* data_output, size_t nframes) {
	uint16_t dma_write_offset = getDMAInPos();
	diff_dma_in = codecBuffers.out_buffer.getAvailableWriteForDMA(dma_write_offset);
	codecBuffers.in_buffer.readInBuffer(dma_write_offset, (CodecFrame*) data_output, nframes);
}

uint32_t CodecAudio::getDMAOutPos() {
	uint32_t dma_pos;

	if(useTlvAsMclkMaster) {
		dma_pos = codecDamcHATInit.getTxRemainingCount();
	} else {
		dma_pos = codecSTM32F723EDiscoInit.getTxRemainingCount();
	}

	uint16_t dma_read_offset =
	    codecBuffers.out_buffer.getCount() - ((dma_pos + 1) / (codecBuffers.out_buffer.getElementSize() / 2));
	return dma_read_offset;
}

uint32_t CodecAudio::getDMAInPos() {
	uint32_t dma_pos;

	if(useTlvAsMclkMaster) {
		dma_pos = codecDamcHATInit.getRxRemainingCount();
	} else {
		dma_pos = codecSTM32F723EDiscoInit.getRxRemainingCount();
	}

	uint16_t dma_write_offset =
	    codecBuffers.in_buffer.getCount() - ((dma_pos + 1) / (codecBuffers.in_buffer.getElementSize() / 2));
	return dma_write_offset;
}

bool CodecAudio::onFastTimer() {
	return false;
}

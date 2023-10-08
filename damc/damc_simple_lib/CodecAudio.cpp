#include "CodecAudio.h"
#include <spdlog/spdlog.h>

#include <assert.h>
#include <math.h>
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_audio.h>
#include <string.h>

CodecAudio CodecAudio::instance;

CodecAudio::CodecAudio() {}

void CodecAudio::start() {
	// 91 gives 0dB HPOUT1L_VOL
	// 80 gives 0dB AIF1ADC1L_VOL
	BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1, OUTPUT_DEVICE_BOTH, 48000, 16, 2, 91, 80);

	// Size in bytes
	BSP_AUDIO_OUT_Play((uint16_t*) out_buffer.getBuffer(), out_buffer.getSize());

	// Size in words (16 bits)
	BSP_AUDIO_IN_Record((uint16_t*) in_buffer.getBuffer(), in_buffer.getSize() / 2);
}

volatile uint32_t diff_dma_out;
void CodecAudio::processAudioInterleavedOutput(const int16_t* data_input, size_t nframes) {
	uint16_t dma_read_offset = getDMAOutPos();
	diff_dma_out = out_buffer.getAvailableReadForDMA(dma_read_offset);
	out_buffer.writeOutBuffer(dma_read_offset, (CodecFrame*) data_input, nframes);
}

volatile uint32_t diff_dma_in;
void CodecAudio::processAudioInterleavedInput(int16_t* data_output, size_t nframes) {
	uint16_t dma_write_offset = getDMAInPos();
	diff_dma_in = out_buffer.getAvailableWriteForDMA(dma_write_offset);
	in_buffer.readInBuffer(dma_write_offset, (CodecFrame*) data_output, nframes);
}

uint32_t CodecAudio::getDMAOutPos() {
	uint32_t dma_pos = BSP_AUDIO_OUT_GetRemainingCount();
	uint16_t dma_read_offset = out_buffer.getCount() - ((dma_pos + 1) / (out_buffer.getElementSize() / 2));
	return dma_read_offset;
}

uint32_t CodecAudio::getDMAInPos() {
	uint32_t dma_pos = BSP_AUDIO_IN_GetRemainingCount();
	uint16_t dma_write_offset = in_buffer.getCount() - ((dma_pos + 1) / (in_buffer.getElementSize() / 2));
	return dma_write_offset;
}

bool CodecAudio::onFastTimer() {
	return false;
}

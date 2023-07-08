#include "CodecAudio.h"
#include <spdlog/spdlog.h>

#include <math.h>
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_audio.h>
#include <string.h>
#include <assert.h>

CodecAudio CodecAudio::instance;

CodecAudio::CodecAudio() {
	memset(out_buffer.data(), 0, out_buffer.size() * sizeof(out_buffer[0]));
	memset(in_buffer.data(), 0, in_buffer.size() * sizeof(in_buffer[0]));
}

void CodecAudio::start() {
	BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_1, OUTPUT_DEVICE_HEADPHONE1, 48000, 16, 2, 40, 100);

	BSP_AUDIO_OUT_Play((uint16_t*)out_buffer.data(), out_buffer.size() * sizeof(out_buffer[0]));
	BSP_AUDIO_IN_Record((uint16_t*)in_buffer.data(), in_buffer.size()*2);
}

void CodecAudio::processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes) {
	writeOutBuffer((uint32_t*)data_output, nframes);
	readInBuffer((uint32_t*)data_output, nframes);
}

volatile uint32_t diff_dma;
volatile uint32_t diff_dma_out;
volatile uint32_t previous_dma_read_offset;
volatile uint32_t max_dma_pos = 5;
volatile uint32_t min_dma_pos = 5;
volatile uint32_t write_size;
void CodecAudio::writeOutBuffer(const uint32_t* data, size_t nframes) {
  uint16_t start = out_write_offset;
  uint16_t size = nframes;

  uint32_t dma_pos = BSP_AUDIO_OUT_GetRemainingCount();
  if(dma_pos > max_dma_pos)
	  max_dma_pos = dma_pos;
  if(dma_pos < min_dma_pos)
	  min_dma_pos = dma_pos;

  uint16_t dma_read_offset = out_buffer.size() - ((dma_pos+1)/2);
  uint16_t max_size = (dma_read_offset - out_write_offset - 1 + out_buffer.size()) % out_buffer.size();

  diff_dma_out = (out_buffer.size() + CodecAudio::instance.out_write_offset - dma_read_offset) % out_buffer.size();
  diff_dma = (out_buffer.size() + dma_read_offset - previous_dma_read_offset) % out_buffer.size();
  previous_dma_read_offset = dma_read_offset;


  if(size > max_size)
	size = max_size;

  write_size = size;

  uint16_t end = (start + size) % out_buffer.size();

  assert(start < out_buffer.size());
  assert(end < out_buffer.size());


  if(end < start) {
	// Copy between start and end of buffer
	uint16_t first_chunk_size = out_buffer.size() - start;
	memcpy(&out_buffer[start], data, first_chunk_size * sizeof(out_buffer[0]));

	// then between begin of buffer and end
	memcpy(&out_buffer[0], &data[first_chunk_size], end * sizeof(out_buffer[0]));
  } else {
	// Copy from start to end
	memcpy(&out_buffer[start], data, (end - start) * sizeof(out_buffer[0]));
  }

  assert(end < out_buffer.size());
  SCB_CleanDCache_by_Addr((uint32_t*)out_buffer.data(), out_buffer.size() * sizeof(out_buffer[0]));
  out_write_offset = end;
}

volatile uint32_t in_max_dma_pos = 5;
volatile uint32_t in_min_dma_pos = 5;
volatile uint32_t in_dma_pos = 5;
void CodecAudio::readInBuffer(uint32_t* data, size_t nframes) {
  uint16_t start = in_read_offset;

  uint32_t dma_pos = BSP_AUDIO_IN_GetRemainingCount();
  if(dma_pos > in_max_dma_pos)
	  in_max_dma_pos = dma_pos;
  if(dma_pos < in_min_dma_pos)
	  in_min_dma_pos = dma_pos;
  in_dma_pos = dma_pos;

  uint16_t dma_write_offset = in_buffer.size() - ((dma_pos+1)/2);
  uint16_t end = dma_write_offset;
  uint16_t size = (end - start + in_buffer.size()) % in_buffer.size();

  assert(start < in_buffer.size());
  assert(end < in_buffer.size());

  if(size > nframes) {
	size = nframes;
	end = (in_read_offset + size) % in_buffer.size();
  }
  assert(end < in_buffer.size());

  uint16_t total_size = 0;

  SCB_InvalidateDCache_by_Addr((uint32_t*)in_buffer.data(), in_buffer.size() * sizeof(in_buffer[0]));

  if(end < start) {
	// Copy between start and end of buffer
	uint16_t first_chunk_size = in_buffer.size() - start;
	memcpy(data, &in_buffer[start], first_chunk_size * sizeof(in_buffer[0]));

	// then between begin of buffer and end
	memcpy(&data[first_chunk_size], &in_buffer[0], end * sizeof(in_buffer[0]));

	total_size = first_chunk_size + end;
	assert(total_size <= nframes);
  } else {
	// Copy from start to end
	memcpy(data, &in_buffer[start], (end - start) * sizeof(in_buffer[0]));
	total_size = end - start;
	assert(total_size <= nframes);
  }

  // If not enough data to fill the buffer, add samples using
  // the same value as the last one.
  uint32_t fill_sample = total_size > 0 ? data[total_size-1] : 0;
  for(size_t i = total_size; i < nframes; i++) {
	  data[i] = fill_sample;
  }

  in_read_offset = end;
}

bool CodecAudio::onFastTimer() {
  uint16_t dma_read_offset = out_buffer.size() - ((BSP_AUDIO_OUT_GetRemainingCount()+1)/2);
  uint16_t start = out_write_offset;
  uint16_t end = (out_buffer.size() + dma_read_offset - 1) % out_buffer.size();

  if(end < start) {
	// Copy between start and end of buffer
	uint16_t first_chunk_size = out_buffer.size() - start;
	memset(&out_buffer[start], 0, first_chunk_size * sizeof(out_buffer[0]));

	// then between begin of buffer and end
	memset(&out_buffer[0], 0, end * sizeof(out_buffer[0]));
  } else {
	// Copy from start to end
	memset(&out_buffer[start], 0, (end - start) * sizeof(out_buffer[0]));
  }

  return false;
}


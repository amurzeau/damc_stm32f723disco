#pragma once

#include <array>
#include <stdint.h>
#include <vector>

#include <spdlog/spdlog.h>

#include <assert.h>
#include <math.h>
#include <stm32f7xx.h>
#include <string.h>

template<typename T, int N, bool do_manage_cache> class CircularBuffer {
public:
	CircularBuffer();

	void writeOutBuffer(uint32_t dma_read_offset, const T* data, size_t word_size);
	size_t readInBuffer(uint32_t dma_write_offset, T* data, size_t word_size);
	void* getBuffer() { return &buffer[0]; }
	constexpr size_t getElementSize() { return sizeof(buffer[0]); }
	size_t getSize() { return buffer.size() * getElementSize(); }
	size_t getCount() { return buffer.size(); }
	size_t getAvailableReadForDMA(uint32_t dma_read_offset) {
		return (buffer.size() + out_write_offset - dma_read_offset) % buffer.size();
	}
	size_t getAvailableWriteForDMA(uint32_t dma_write_offset) {
		return (buffer.size() + dma_write_offset - in_read_offset) % buffer.size();
	}
	size_t getWritePos() { return out_write_offset; }
	size_t getReadPos() { return in_read_offset; }

private:
	// Double buffer and dual channel
	// This must be 32 byte aligned for cache handling
	std::array<T, 48 * N> buffer __attribute__((aligned(32)));
	size_t out_write_offset;
	size_t in_read_offset;
};

template<typename T, int N, bool do_manage_cache> CircularBuffer<T, N, do_manage_cache>::CircularBuffer() {
	memset(buffer.data(), 0, buffer.size() * sizeof(buffer[0]));
}

template<typename T, int N, bool do_manage_cache>
void CircularBuffer<T, N, do_manage_cache>::writeOutBuffer(uint32_t dma_read_offset, const T* data, size_t nframes) {
	uint16_t start = out_write_offset;
	uint16_t size = nframes;

	uint16_t max_size = (dma_read_offset - out_write_offset - 1 + buffer.size()) % buffer.size();

	if(size > max_size)
		size = max_size;

	uint16_t end = (start + size) % buffer.size();

	assert(start < buffer.size());
	assert(end < buffer.size());

	if(end < start) {
		// Copy between start and end of buffer
		uint16_t first_chunk_size = buffer.size() - start;
		memcpy(&buffer[start], data, first_chunk_size * sizeof(buffer[0]));

		// then between begin of buffer and end
		memcpy(&buffer[0], &data[first_chunk_size], end * sizeof(buffer[0]));
	} else {
		// Copy from start to end
		memcpy(&buffer[start], data, (end - start) * sizeof(buffer[0]));
	}

	assert(end < buffer.size());
	if(do_manage_cache) {
		SCB_CleanDCache_by_Addr((uint32_t*) buffer.data(), buffer.size() * sizeof(buffer[0]));
	} else {
		__DSB();
	}
	assert(out_write_offset == start);
	out_write_offset = end;
}

template<typename T, int N, bool do_manage_cache>
size_t CircularBuffer<T, N, do_manage_cache>::readInBuffer(uint32_t dma_write_offset, T* data, size_t nframes) {
	uint16_t start = in_read_offset;

	uint16_t end = dma_write_offset;
	uint16_t size = (end - start + buffer.size()) % buffer.size();

	assert(start < buffer.size());
	assert(end < buffer.size());

	if(size > nframes) {
		size = nframes;
		end = (start + size) % buffer.size();
	}
	assert(end < buffer.size());

	uint16_t total_size = 0;

	if(do_manage_cache) {
		SCB_InvalidateDCache_by_Addr((uint32_t*) buffer.data(), buffer.size() * sizeof(buffer[0]));
	}

	if(end < start) {
		// Copy between start and end of buffer
		uint16_t first_chunk_size = buffer.size() - start;
		memcpy(data, &buffer[start], first_chunk_size * sizeof(buffer[0]));

		// then between begin of buffer and end
		memcpy(&data[first_chunk_size], &buffer[0], end * sizeof(buffer[0]));

		total_size = first_chunk_size + end;
		assert(total_size <= nframes);
	} else {
		// Copy from start to end
		memcpy(data, &buffer[start], (end - start) * sizeof(buffer[0]));
		total_size = end - start;
		assert(total_size <= nframes);
	}

	// If not enough data to fill the buffer, add samples using
	// the same value as the last one.
	T fill_sample = total_size > 0 ? data[total_size - 1] : T{};
	for(size_t i = total_size; i < nframes; i++) {
		data[i] = fill_sample;
	}

	if(!do_manage_cache) {
		__DSB();
	}
	assert(in_read_offset == start);
	in_read_offset = end;

	return size;
}

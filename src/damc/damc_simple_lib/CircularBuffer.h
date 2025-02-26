#pragma once

#include <array>
#include <stdint.h>

#include <spdlog/spdlog.h>

#include <assert.h>
#include <math.h>
#include <stm32f7xx.h>
#include <string.h>

template<typename T, int N, bool do_manage_cache> class CircularBuffer {
public:
	CircularBuffer();

	size_t writeOutBuffer(uint32_t dma_read_offset, const T* data, size_t word_size);
	size_t readInBuffer(uint32_t dma_write_offset, T* data, size_t word_size);
	void* getBuffer() { return &buffer[0]; }
	constexpr size_t getElementSize() { return sizeof(buffer[0]); }
	size_t getSize() { return buffer.size() * getElementSize(); }
	size_t getCount() { return buffer.size(); }
	size_t getAvailableReadForDMA(uint32_t dma_read_offset) {
		return (buffer.size() + out_write_offset.offset - dma_read_offset) % buffer.size();
	}
	size_t getAvailableWriteForDMA(uint32_t dma_write_offset) {
		return (buffer.size() + in_read_offset.offset - dma_write_offset - 1) % buffer.size();
	}
	size_t getWritePos() { return out_write_offset.offset; }
	size_t getReadPos() { return in_read_offset.offset; }

	// Call to reset buffer to a given size when the read pos is controlled in another thread
	void resetWritePos(size_t available_space) {
		assert(available_space < buffer.size());
		// Configure buffers so there are available_space space for reading of empty data
		out_write_offset.offset = (uint16_t) (available_space + in_read_offset.offset) % buffer.size();
		silenceAvailableSamples();
	}

	// Call to reset buffer to a given size when the write pos is controlled in another thread
	void resetReadPos(size_t available_space) {
		assert(available_space < buffer.size());
		// Configure buffers so there are available_space space for reading of empty data
		in_read_offset.offset = (uint16_t) (buffer.size() + out_write_offset.offset - available_space) % buffer.size();
		silenceAvailableSamples();
	}

	void silenceAvailableSamples() {
		uint16_t start = in_read_offset.offset;
		uint16_t end = out_write_offset.offset;

		if(end < start) {
			// Clear between start and end of buffer
			uint16_t first_chunk_size = buffer.size() - start;
			memset(&buffer[start], 0, first_chunk_size * sizeof(buffer[0]));

			// then between begin of buffer and end
			memset(&buffer[0], 0, end * sizeof(buffer[0]));
		} else {
			// Clear from start to end
			memset(&buffer[start], 0, (end - start) * sizeof(buffer[0]));
		}
	}

	void resetBufferProcessedFlag() {
		assert(out_write_offset.offset < buffer.size());
		assert(in_read_offset.offset < buffer.size());
		__DSB();
		out_write_offset.buffer_processed = false;
		in_read_offset.buffer_processed = false;
		__DSB();
		assert(out_write_offset.offset < buffer.size());
		assert(in_read_offset.offset < buffer.size());
	}

	uint16_t isBufferWritten() { return out_write_offset.buffer_processed; }
	uint16_t isBufferRead() { return in_read_offset.buffer_processed; }

private:
	union Offset {
		struct {
			uint16_t buffer_processed : 16;
			uint16_t offset : 16;
		};
		uint32_t raw;
	};
	// Double buffer and dual channel
	// This must be 32 byte aligned for cache handling
	std::array<T, 48 * N> buffer __attribute__((aligned(32)));
	Offset out_write_offset;
	Offset in_read_offset;
};

template<typename T, int N, bool do_manage_cache> CircularBuffer<T, N, do_manage_cache>::CircularBuffer() {
	memset(buffer.data(), 0, buffer.size() * sizeof(buffer[0]));
}

template<typename T, int N, bool do_manage_cache>
size_t CircularBuffer<T, N, do_manage_cache>::writeOutBuffer(uint32_t dma_read_offset, const T* data, size_t nframes) {
	uint16_t start = out_write_offset.offset;
	uint16_t size = nframes;

	uint16_t max_size = (dma_read_offset - out_write_offset.offset - 1 + buffer.size()) % buffer.size();

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
	assert(out_write_offset.offset == start);

	// Atomically update both the write pointer and the buffer processed flag
	// using a single 32 bits write.
	out_write_offset.raw = Offset{true, end}.raw;

	return size;
}

template<typename T, int N, bool do_manage_cache>
size_t CircularBuffer<T, N, do_manage_cache>::readInBuffer(uint32_t dma_write_offset, T* data, size_t nframes) {
	uint16_t start = in_read_offset.offset;

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
	assert(in_read_offset.offset == start);

	// Atomically update both the read pointer and the buffer processed flag
	// using a single 32 bits write.
	in_read_offset.raw = Offset{true, end}.raw;

	return size;
}

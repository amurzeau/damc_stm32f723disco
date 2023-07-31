#pragma once

#include "CircularBuffer.h"
#include <array>
#include <stdint.h>
#include <vector>

class CodecAudio {
public:
	CodecAudio();

	void start();

	void processAudioInterleavedOutput(const int16_t* data_input, size_t nframes);
	void processAudioInterleavedInput(int16_t* data_output, size_t nframes);

	uint32_t getDMAPos();

	bool onFastTimer();

	static CodecAudio instance;

protected:
	void writeOutBuffer(const uint32_t* data, size_t word_size);
	void readInBuffer(uint32_t* data, size_t word_size);

private:
	CircularBuffer<2> out_buffer;
	CircularBuffer<2> in_buffer;
};

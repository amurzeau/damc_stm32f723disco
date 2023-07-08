#pragma once

#include <array>
#include <stdint.h>
#include <vector>

class CodecAudio {
public:
	CodecAudio();

	void start();

	void processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes);

	bool onFastTimer();

	static CodecAudio instance;

protected:
	void writeOutBuffer(const uint32_t* data, size_t word_size);
	void readInBuffer(uint32_t* data, size_t word_size);

private:
	// Double buffer and dual channel
	std::array<uint32_t, 48*3> out_buffer;
	std::array<uint32_t, 48*3> in_buffer;
	size_t out_write_offset;
	size_t in_read_offset;
};

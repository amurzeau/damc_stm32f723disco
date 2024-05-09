#pragma once

#include "CircularBuffer.h"
#include "CodecDamcHATInit.h"
#include "CodecSTM32F723EDiscoInit.h"
#include <array>
#include <stdint.h>
#include <vector>

class CodecAudio {
public:
	struct CodecFrame {
		int32_t headphone[2];
	};

	CodecAudio();

	void start();

	void processAudioInterleavedOutput(const CodecFrame* data_input, size_t nframes);
	void processAudioInterleavedInput(CodecFrame* data_output, size_t nframes);

	uint32_t getDMAOutPos();
	uint32_t getDMAInPos();

	bool onFastTimer();

	static CodecAudio instance;

protected:
	void writeOutBuffer(const uint32_t* data, size_t word_size);
	void readInBuffer(uint32_t* data, size_t word_size);

private:
	struct CodecBuffers {
		CircularBuffer<CodecFrame, 2, true> out_buffer;
		CircularBuffer<CodecFrame, 2, true> in_buffer;
	};

	CodecBuffers codecBuffers;

	CodecSTM32F723EDiscoInit codecSTM32F723EDiscoInit;
	CodecDamcHATInit codecDamcHATInit;
	bool useTlvAsMclkMaster;
};

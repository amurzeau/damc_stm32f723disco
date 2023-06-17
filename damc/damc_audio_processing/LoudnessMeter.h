#pragma once

#include "BiquadFilter.h"
#include <stddef.h>
#include <vector>

/**
 * @brief Loudness meter.
 * Measure loudness according to ITU-R BS.1770-4
 * This consists of doing this:
 * - Apply stage 1 pre-filter: +4 dB above 1500 Hz, no change below 1500 Hz
 * - Apply stage 2 pre-filter: 60 Hz (-3dB) 2nd order high pass filter
 * - Mean Square all samples in a 400 ms moving interval (gate)
 * - Sum all channels
 * - LKFS = -0.691 + 10*log10(result)
 *
 * Note: there is no root applied, instead the root is done by dividing log10 by 2 (doing 10*log10 instead of 20*log10)
 */

class LoudnessMeter {
public:
	LoudnessMeter();

	void reset(float fs);
	void processSamples(const float* input, size_t count);
	void processOneSample(float input);

	// Returns LKFS
	float getLoudness();

protected:
	void setParameters(unsigned int delay);

private:
	BiquadFilter stage1Filter;
	BiquadFilter stage2HighPassFilter;
	// a moving gate of 400ms filtered and squared samples
	std::vector<int64_t> gatedSquaredSamples;
	int64_t squaredAverage;

	size_t inputIndex;
	size_t outputIndex;
	size_t power2Size;
	float squaredAverageScale;
	size_t delaySamples;
	float fs = 48000;
};

#include "LoudnessMeter.h"

#include <math.h>
#include <string.h>

LoudnessMeter::LoudnessMeter() {
	stage1Filter.computeFilter(true, FilterType::HighShelf, fs, 1500, 4, 0.7071);
	stage2HighPassFilter.computeFilter(true, FilterType::HighPass, fs, 38, 0, 0.5);
}

void LoudnessMeter::reset(float fs) {
	this->fs = fs;
	std::fill(this->gatedSquaredSamples.begin(), this->gatedSquaredSamples.end(), 0.f);
	// 400ms moving gate
	setParameters(0.4 * fs);
	stage1Filter.computeFilter(true, FilterType::HighShelf, 1500, fs, 4, 0.7071);
	stage2HighPassFilter.computeFilter(true, FilterType::HighPass, 38, fs, 0, 0.5);
}

void LoudnessMeter::processSamples(const float* input, size_t count) {
	for(size_t i = 0; i < count; i++) {
		processOneSample(input[i]);
	}
}

void LoudnessMeter::processOneSample(float input) {
	// part 1: computed for each samples

	// stage 1 filter
	float sample = stage1Filter.put(input);

	// stage 2 filter
	sample = stage2HighPassFilter.put(sample);

	// square samples
	sample = sample * sample;

	// sum squared samples over the 400ms interval

	// note: we use integers instead of float to have perfect numerical sum in squaredAverage
	int64_t integerSquaredSample = sample * squaredAverageScale;

	gatedSquaredSamples[inputIndex] = integerSquaredSample;
	int64_t output = gatedSquaredSamples[outputIndex];
	inputIndex = (inputIndex + 1) & ((1 << power2Size) - 1);
	outputIndex = (outputIndex + 1) & ((1 << power2Size) - 1);

	squaredAverage = squaredAverage - output + integerSquaredSample;
}

float LoudnessMeter::getLoudness() {
	// part 2: computed only when we need the resulting loudness

	// move back to floats
	float squaredAverageFloat = 2.0f * squaredAverage / squaredAverageScale;

	// divide the result by the number of samples in the interval
	// to get the mean square of samples

	float output = squaredAverageFloat / delaySamples;

	// move to log scale to get LUKS
	output = -0.691 + 10 * log10f(output);

	return output;
}

void LoudnessMeter::setParameters(unsigned int delay) {
	size_t powerOfTwo;
	int targetDelay = delay;

	this->delaySamples = delay;

	powerOfTwo = 0;
	while((1 << powerOfTwo) < (targetDelay + 1))
		powerOfTwo++;
	this->power2Size = powerOfTwo;
	this->gatedSquaredSamples.resize(1 << power2Size, 0);
	this->outputIndex = (inputIndex + this->gatedSquaredSamples.size() - targetDelay) & ((1 << power2Size) - 1);
	this->squaredAverage = 0;
	this->squaredAverageScale = INT64_MAX >> power2Size;
}

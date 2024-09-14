#include "ResamplingFilter.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

bool ResamplingFilter::initialized;
std::array<float, 8192> ResamplingFilter::optimized_coefs;

void ResamplingFilter::reset(float fs) {
	if(!initialized) {
		initialized = true;

		for(size_t k = 0; k < oversamplingRatio; k++) {
			for(size_t i = 0; i < tapPerSample; i++) {
				optimized_coefs[i + (oversamplingRatio - 1 - k) * tapPerSample] =
				    coefs[k + i * oversamplingRatio] * oversamplingRatio;
			}
		}
	}

	baseSamplingRate = fs;
	targetSamplingRate = fs;
	currentPos = 0;
	previousDelay = oversamplingRatio - 1;
	history.fill(0);
}

void ResamplingFilter::put(float sample) {
	history[currentPos] = sample;
	currentPos = (currentPos - 1) % history.size();
}

void ResamplingFilter::get(float* __restrict output, size_t* __restrict outSize, float period) {
	size_t newOutSize = *outSize;
	float delay = previousDelay;

	while(delay >= 0) {
		output[newOutSize] = getZeroOrderHoldInterpolatedPoint(delay);
		newOutSize++;
		delay -= period;
	}

	previousDelay = delay + oversamplingRatio;
	*outSize = newOutSize;
}

int ResamplingFilter::processSamples(float* __restrict output,
                                     size_t* __restrict outSize,
                                     const float* __restrict input,
                                     size_t count) {
	size_t newOutSize = 0;

	for(size_t i = 0; i < count; i++) {
		put(input[i]);
		get(output, &newOutSize, downsamplingRatio);
	}

	*outSize = newOutSize;

	return 0;
}

int ResamplingFilter::getNextOutputSize() {
	int iterations = 0;
	float delay = previousDelay;

	while(delay >= 0) {
		delay -= downsamplingRatio;
		iterations++;
	}

	return iterations;
}

size_t ResamplingFilter::getMaxRequiredOutputSize(size_t count) {
	return ceil(count * oversamplingRatio / downsamplingRatio);
}

size_t ResamplingFilter::getMinRequiredOutputSize(size_t count) {
	return floor(count * oversamplingRatio / downsamplingRatio);
}

void ResamplingFilter::setClockDrift(float drift) {
	float newRatio = oversamplingRatio * baseSamplingRate / targetSamplingRate / (1.0 + (float) drift);
	if(newRatio >= 1)
		downsamplingRatio = newRatio;
}

float ResamplingFilter::getClockDrift() {
	return (oversamplingRatio * baseSamplingRate / targetSamplingRate / downsamplingRatio) - 1.0;
}

void ResamplingFilter::setSourceSamplingRate(float samplingRate) {
	baseSamplingRate = samplingRate;
	downsamplingRatio = oversamplingRatio * baseSamplingRate / targetSamplingRate;
}

float ResamplingFilter::getSourceSamplingRate() {
	return baseSamplingRate;
}

void ResamplingFilter::setTargetSamplingRate(float samplingRate) {
	targetSamplingRate = samplingRate;
	downsamplingRatio = oversamplingRatio * baseSamplingRate / targetSamplingRate;
}

float ResamplingFilter::getTargetSamplingRate() {
	return targetSamplingRate;
}

float ResamplingFilter::getLinearInterpolatedPoint(float delay) const {
	int x = int(delay + 2) - 2;

	float rem = delay - x;

	if(rem == 0)
		return getOnePoint(x);

	float v1 = getOnePoint(x);
	float v2 = getOnePoint(x + 1);
	return v1 + rem * (v2 - v1);
}

float ResamplingFilter::getZeroOrderHoldInterpolatedPoint(float delay) const {
	return getOnePoint(int(delay));
}

float ResamplingFilter::getOnePoint(unsigned int delay) const {
	float sum = 0;
	unsigned int index = currentPos + (delay / oversamplingRatio);
	delay %= oversamplingRatio;
	const float* coefs_to_use = &optimized_coefs[delay * tapPerSample];
	unsigned int k;

	size_t firstLoop = std::min(tapPerSample, history.size() - index);

	for(k = 0; k < firstLoop; k++) {
		sum += history[index + k] * coefs_to_use[k];
	}

	index = (index + k) % history.size();

	for(; k < tapPerSample; k++, index++) {
		sum += history[index] * coefs_to_use[k];
	}

	return sum;
}

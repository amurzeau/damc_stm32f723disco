#include "LoudnessMeter.h"

#include "MathUtils.h"
#include <fastapprox/fastexp.h>
#include <fastapprox/fastlog.h>
#include <math.h>
#include <string.h>

LoudnessMeter::LoudnessMeter() : squaredAverage(0), fs(48000) {
	reset(fs);
	setGateLevel(-80);
}

void LoudnessMeter::reset(float fs) {
	this->fs = fs;
	stage1Filter.computeFilter(true, FilterType::HighShelf, 1500, fs, 4, 0.7071);
	stage2HighPassFilter.computeFilter(true, FilterType::HighPass, 38, fs, 0, 0.5);
	squaredAverage = 0;
	// This value match the 400ms moving average filter
	setAveragingTime(0.144686434631885);
}

void LoudnessMeter::setAveragingTime(float tauSecond) {
	double alpha = expf(-1.f / (tauSecond * 48000.f));
	averageAlpha = alpha;
	oneMinusAverageAlpha = 1 - alpha;
}

void LoudnessMeter::setGateLevel(float levelDb) {
	gateLevel = powf(10, levelDb / 20);
}

void LoudnessMeter::processSamples(const float* input, size_t count) {
	for(size_t i = 0; i < count; i++) {
		processOneSample(input[i]);
	}
}

void LoudnessMeter::processOneSample(float input) {
	// part 1: computed for each samples

	// Gate-out silence
	if(fabsf(input) < gateLevel) {
		return;
	}

	// stage 1 filter
	float sample = stage1Filter.put(input);

	// stage 2 filter
	sample = stage2HighPassFilter.put(sample);

	// square samples
	sample = sample * sample;

	// average squared samples using an IIR filter instead of moving average of 400ms to reduce memory usage
	squaredAverage = averageAlpha * squaredAverage + oneMinusAverageAlpha * sample;
}

float LoudnessMeter::linearToLufs(float linearMeanSquareChannelSum) {
	// part 2: computed only when we need the resulting loudness

	// move to log scale to get LUKS
	// Note: we divide by 2 the log result to do the "square root" in the log domain
	return -0.691 + fastlog2(linearMeanSquareChannelSum) / (LOG10_VALUE_DIV_20 * 2);
}

float LoudnessMeter::lufsToLinear(float lufs) {
	// move to linear scale from LUKS
	return fastpow2((lufs + 0.691) * (LOG10_VALUE_DIV_20 * 2));
}

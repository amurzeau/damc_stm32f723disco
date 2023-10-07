#include "DelayFilter.h"
#include <stm32f7xx.h>

#include <math.h>
#include <string.h>

DelayFilter::DelayFilter() {
	this->inputIndex = 0;
	this->outputIndex = 0;
	setParameters(0);
}

void DelayFilter::reset() {
	std::fill(this->delayedSamples.begin(), this->delayedSamples.end(), 0);
}

void DelayFilter::processSamples(float* samples, size_t count) {
	if(this->delay > 0) {
		for(size_t i = 0; i < count; i++) {
			samples[i] = processOneSample(samples[i]);
		}
	}
}

float DelayFilter::processOneSample(float input) {
	delayedSamples[inputIndex] = input;
	float output = delayedSamples[outputIndex];
	inputIndex = (inputIndex + 1) & ((1 << power2Size) - 1);
	outputIndex = (outputIndex + 1) & ((1 << power2Size) - 1);

	return output;
}

void DelayFilter::setParameters(unsigned int delay) {
	size_t powerOfTwo;
	int targetDelay = delay;

	powerOfTwo = 0;
	while((1 << powerOfTwo) < (targetDelay + 1))
		powerOfTwo++;

	// Disable delay processing while we are readjusting buffers and indexes
	this->delay = 0;
	__DSB();

	this->power2Size = powerOfTwo;
	this->delayedSamples.resize(1 << powerOfTwo, 0);

	outputIndex = (inputIndex + this->delayedSamples.size() - targetDelay) & ((1 << powerOfTwo) - 1);

	__DSB();
	this->delay = delay;
}

#include "DelayFilter.h"
#include <stm32f7xx.h>

#include <math.h>
#include <string.h>

DelayFilter::DelayFilter() : delayedSamples(nullptr) {
	this->inputIndex = 0;
	this->outputIndex = 0;
	setParameters(0);
}

void DelayFilter::reset() {}

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
	unsigned int targetDelay = delay;

	powerOfTwo = 0;
	while((1u << powerOfTwo) < (targetDelay + 1u))
		powerOfTwo++;

	// Disable delay processing while we are readjusting buffers and indexes
	this->delay = 0;
	__DSB();

	if(this->delayedSamples) {
		free(this->delayedSamples);
		this->delayedSamples = nullptr;
	}

	size_t arraySize = 1 << powerOfTwo;

	if(delay > 0) {
		this->delayedSamples = (float*) calloc(arraySize, sizeof(float));
		if(this->delayedSamples == nullptr) {
			// Not enough memory
			return;
		}
	}

	this->power2Size = powerOfTwo;
	this->outputIndex = (this->inputIndex + arraySize - targetDelay) & (arraySize - 1);

	__DSB();
	this->delay = delay;
}

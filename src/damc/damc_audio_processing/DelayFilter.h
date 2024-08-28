#pragma once

#include <stddef.h>
#include <vector>

class DelayFilter {
public:
	DelayFilter();

	void reset();
	void processSamples(float* samples, size_t count);
	float processOneSample(float input);

	void setParameters(unsigned int delay);
	void getParameters(unsigned int& delay) { delay = this->delay; }

private:
	float* delayedSamples;
	unsigned int delay;
	size_t inputIndex;
	size_t outputIndex;
	size_t power2Size;
};

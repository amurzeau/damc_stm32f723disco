#include "DitheringFilter.h"

#include <math.h>
#include <string.h>
#include <time.h>

DitheringFilter::DitheringFilter() : dither1(0, 1), dither2(-1, 0) {
	randGenerator.seed(time(nullptr));
	this->scale = 1;
	this->bitReduction = 0;
	bitRatio = 0;
}

void DitheringFilter::reset(float) {
	previousRandom = 0;
	previousQuantizationError = 0;
}

void DitheringFilter::processSamples(float* samples, size_t count) {
	if(bitReduction) {
		float quantizationError = previousQuantizationError;
		for(size_t i = 0; i < count; i++) {
			float r = (dither1(randGenerator) - 0.5) / bitRatio;
			float dither = previousRandom - r;
			previousRandom = r;
			if(scale == 0)
				dither = 0;
			float nonQuantizedOutput = samples[i] - dither /*+ scale * quantizationError*/;
			samples[i] = floor(nonQuantizedOutput * bitRatio) / bitRatio;
			quantizationError = samples[i] - nonQuantizedOutput;
		}
		previousQuantizationError = quantizationError;
	}
}

void DitheringFilter::setParameters(float scale, int bitReduction) {
	this->scale = scale;
	this->bitReduction = bitReduction;
	if(bitReduction > 0)
		bitRatio = pow(2, bitReduction - 1);
	else
		bitRatio = 0;
}

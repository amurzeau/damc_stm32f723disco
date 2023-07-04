#pragma once

#include <random>
#include <stddef.h>
#include <vector>

class DitheringFilter {
public:
	DitheringFilter();

	void reset(float fs);
	void processSamples(float* samples, size_t count);

	void setParameters(float scale, int bitReduction);
	void getParameters(float& scale, int& bitReduction) {
		scale = this->scale;
		bitReduction = this->bitReduction;
	}

private:
	float scale;
	int bitReduction;
	float bitRatio;
	float previousQuantizationError;
	float previousRandom;
	std::uniform_real_distribution<float> dither1;
	std::uniform_real_distribution<float> dither2;
	std::mt19937_64 randGenerator;
};

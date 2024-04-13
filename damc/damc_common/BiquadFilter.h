#pragma once

#include <complex>
#include <stddef.h>

enum class FilterType {
	None,
	LowPass,
	HighPass,
	BandPassConstantSkirt,
	BandPassConstantPeak,
	Notch,
	AllPass,
	Peak,
	LowShelf,
	HighShelf
};

class BiquadFilter {
public:
	void init(const float a_coefs[3], const float b_coefs[3]);
	void update(const float a_coefs[3], const float b_coefs[3]);
	float put(float input);

	static void computeFilter(bool enabled,
	                          FilterType filterType,
	                          float f0,
	                          float fs,
	                          float gain,
	                          float Q,
	                          float a_coefs[3],
	                          float b_coefs[3]);
	void computeFilter(bool enabled, FilterType filterType, float f0, float fs, float gain, float Q);
	std::complex<float> getResponse(float f0, float fs);

private:
	float b_coefs[3] = {};
	float a_coefs[2] = {};

	float input1 = 0;
	float input2 = 0;
	float output1 = 0;
	float output2 = 0;
};

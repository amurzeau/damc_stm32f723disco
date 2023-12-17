#include "EqFilter.h"

#include <algorithm>
#include <cmath>
#include <string.h>

EqFilter::EqFilter(OscContainer* parent, const std::string_view& name)
    : OscContainer(parent, name),
      enabled(this, "enable", false),
      filterType(this, "type", (int32_t) FilterType::None),
      f0(this, "f0", 1000),
      gain(this, "gain", 0),
      Q(this, "Q", 0.5) {
	auto onChangeCallback = [this](auto) { computeFilter(); };
	enabled.addChangeCallback(onChangeCallback);
	filterType.addChangeCallback(onChangeCallback);
	f0.addChangeCallback(onChangeCallback);
	gain.addChangeCallback(onChangeCallback);
	Q.addChangeCallback(onChangeCallback);
}

void EqFilter::init(size_t numChannel) {
	biquadFilters.resize(numChannel);
}

void EqFilter::reset(float fs) {
	this->fs = fs;
	computeFilter();
}

void EqFilter::processSamples(float** samples, size_t count) {
	if(enabled) {
		for(size_t channel = 0; channel < biquadFilters.size(); channel++) {
			// Using local variable for biquadFilter allow keeping the state in registers
			// while processing all samples.
			BiquadFilter biquadFilter = biquadFilters[channel];
			float* outputChannel = samples[channel];
			const float* inputChannel = samples[channel];

			for(size_t i = 0; i < count; i++) {
				outputChannel[i] = biquadFilter.put(inputChannel[i]);
			}

			biquadFilters[channel] = biquadFilter;
		}
	}
}

std::complex<float> EqFilter::getResponse(float f0) {
	return biquadFilters.front().getResponse(f0, fs);
}

void EqFilter::computeFilter() {
	float a_coefs[3];
	float b_coefs[3];

	BiquadFilter::computeFilter(enabled, (FilterType) filterType.get(), f0, fs, gain, Q, a_coefs, b_coefs);

	for(BiquadFilter& biquadFilter : biquadFilters)
		biquadFilter.update(a_coefs, b_coefs);
}

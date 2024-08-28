#pragma once

#include "BiquadFilter.h"
#include <Osc/OscContainer.h>
#include <Osc/OscVariable.h>
#include <array>
#include <complex>
#include <stddef.h>

class EqFilter : public OscContainer {
public:
	EqFilter(OscContainer* parent, const std::string_view& name);

	void init(size_t numChannel);
	void reset(float fs);
	void processSamples(float** samples, size_t count);


private:
	OscVariable<bool> enabled;
	OscVariable<int32_t> filterType;
	OscVariable<float> f0;
	float fs = 48000;
	OscVariable<float> gain;
	OscVariable<float> Q;

	std::array<BiquadFilter, 2> biquadFilters;

	void computeFilter();
};

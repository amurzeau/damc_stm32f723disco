#pragma once

#include <Osc/OscContainer.h>
#include <Osc/OscVariable.h>
#include <stddef.h>
#include <array>

class ExpanderFilter : public OscContainer {
public:
	ExpanderFilter(OscContainer* parent);
	void init(size_t numChannel);
	void reset(float fs);
	void processSamples(float** samples, size_t count);

protected:
	float doCompression(float sample, float& y1, float& yL);
	float gainComputer(float sample);
	void levelDetector(float sample, float& y1, float& yL);

private:
	static constexpr size_t numChannel = 2;
	std::array<float, 2> previousPartialGainComputerOutput;
	std::array<float, 2> previousLevelDetectorOutput;

	OscVariable<bool> enable;
	float fs = 48000;
	float alphaR;
	float alphaA;
	OscVariable<float> attackTime;
	OscVariable<float> releaseTime;
	OscVariable<float> threshold;
	OscVariable<float> makeUpGain;
	OscVariable<float> ratio;
	float gainDiffRatio = 0;
	OscVariable<float> kneeWidth;
};

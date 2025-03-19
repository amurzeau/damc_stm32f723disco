#pragma once

#include <array>
#include <stddef.h>

class ResamplingFilter {
public:
	void reset(float fs);
	void put(float sample);
	void get(float* __restrict out, size_t* __restrict outSize, float period);

	int processSamples(float* __restrict output,
	                   size_t* __restrict outSize,
	                   const float* __restrict input,
	                   size_t count);
	int getNextOutputSize();
	size_t getMaxRequiredOutputSize(size_t count);
	size_t getMinRequiredOutputSize(size_t count);

	void setClockDrift(float drift);
	float getClockDrift();

	void setSourceSamplingRate(float samplingRate);
	float getSourceSamplingRate();

	void setTargetSamplingRate(float samplingRate);
	float getTargetSamplingRate();

	static constexpr unsigned int getOverSamplingRatio() { return oversamplingRatio; }
	float getDownSamplingRatio() { return downsamplingRatio; }

protected:
	inline float getLinearInterpolatedPoint(float delay) const;
	inline float getZeroOrderHoldInterpolatedPoint(float delay) const;
	inline float getOnePoint(unsigned int delay) const;

private:
	unsigned int currentPos;
	float previousDelay;
	std::array<float, 256> history;

	float baseSamplingRate = 48000.f;
	float targetSamplingRate = 48000.f;
	float downsamplingRatio = oversamplingRatio;

	static bool initialized;
	static const std::array<float, 8192> coefs;
	static std::array<float, 8192> optimized_coefs;
	static constexpr unsigned int oversamplingRatio = 128;
	static constexpr unsigned int tapPerSample = (unsigned int) coefs.size() / oversamplingRatio;
};

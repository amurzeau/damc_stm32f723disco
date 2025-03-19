#pragma once

#include "BiquadFilter.h"
#include <stddef.h>
#include <vector>

/**
 * @brief Loudness meter.
 * Measure loudness according to ITU-R BS.1770-4
 * https://www.itu.int/dms_pubrec/itu-r/rec/bs/R-REC-BS.1770-4-201510-I!!PDF-E.pdf
 *
 * This consists of doing this:
 * - Apply stage 1 pre-filter: +4 dB above 1500 Hz, no change below 1500 Hz
 * - Apply stage 2 pre-filter: 60 Hz (-3dB) 2nd order high pass filter
 * - Mean Square all samples in a 400 ms moving interval (gate)
 * - Sum all channels
 * - LKFS = -0.691 + 10*log10(result)
 *
 * Note: there is no root applied, instead the root is done by dividing log10 by 2 (doing 10*log10 instead of 20*log10)
 *
 * For memory reason, the mean is done using an IIR filter instead of a moving average over 400ms.
 *
 * The moving average of 400ms is equivalent to a single pole IIR filter with alpha = 0.999856020824846
 * y(n) = alpha * y(n-1) + (1 - alpha) * x(n)
 * Cut-off -3dB at 1.1 Hz
 * Check in matlab or octave:
 *   IIR filter: a=0.999856020824846 ;num = 1-a; den = [1, -a]; freqz(num, den, 192000, "whole", 48000);
 *   Moving average (FIR): s=0.4*48000; freqz(repelem(1/s,s),1,192000, "whole", 48000);
 * Note: beware of wrong computation in programs that don't use sufficient precision (like libreoffice calc)
 */

class LoudnessMeter {
public:
	LoudnessMeter();

	void reset(float fs);
	void processSamples(const float* input, size_t count);
	void processOneSample(float input);

	void setAveragingTime(float tauSecond);
	void setGateLevel(float level);

	// Linear values. The result can be added between channels.
	// To be fed to linearToLuks to get the LUKS loudness value.
	float getSquaredAverage() { return squaredAverage; }

	// Returns LUKS
	static float linearToLufs(float linearMeanSquareChannelSum);
	static float lufsToLinear(float lufs);

private:
	BiquadFilter stage1Filter;
	BiquadFilter stage2HighPassFilter;

	float squaredAverage;
	float fs;

	float averageAlpha;
	float oneMinusAverageAlpha;
	float gateLevel;
};

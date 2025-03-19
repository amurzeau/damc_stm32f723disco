#include "CompressorFilter.h"

#include <MathUtils.h>
#include <algorithm>
#include <assert.h>
#include <fastapprox/fastexp.h>
#include <fastapprox/fastlog.h>
#include <math.h>
#include <string.h>

// Algorithm based on
// https://www.eecs.qmul.ac.uk/~josh/documents/2012/GiannoulisMassbergReiss-dynamicrangecompression-JAES2012.pdf

CompressorFilter::CompressorFilter(OscContainer* parent)
    : OscContainer(parent, "compressorFilter", 9),
      enablePeak(this, "enable", false),
      enableLoudness(this, "enableLoudness", false),
      attackTime(this, "attackTime", 0),
      releaseTime(this, "releaseTime", 2),
      threshold(this, "threshold", -50),
      makeUpGain(this, "makeUpGain", 0),
      ratio(this, "ratio", 1000),
      kneeWidth(this, "kneeWidth", 0),
      lufsTarget(this, "lufsTarget", -14),
      lufsIntegrationTime(this, "lufsIntegTime", 0.144686434631885),
      lufsGate(this, "lufsGate", -80),
      lufsRealtimeLevel(-138),
      lufsMeter(this, "lufsMeter", 0) {
	reset();
	attackTime.addChangeCallback(
	    [this](float oscValue) { alphaA = oscValue != 0 ? expf(-1 / (oscValue * 48000)) : 0; });
	releaseTime.addChangeCallback(
	    [this](float oscValue) { alphaR = oscValue != 0 ? expf(-1 / (oscValue * 48000)) : 0; });
	ratio.addChangeCallback([this](float oscValue) {
		gainDiffRatio = 1 - 1 / oscValue;
		gainDiffRatioInKnee = gainDiffRatio / (2 * kneeWidth);
		reset();
	});
	threshold.addChangeCallback([this](float oscValue) { reset(); });
	kneeWidth.addChangeCallback([this](float oscValue) {
		gainDiffRatioInKnee = gainDiffRatio / (2 * kneeWidth);
		reset();
	});
	lufsIntegrationTime.addChangeCallback([this](float oscValue) {
		for(auto& loudnessMeter : loudnessMeters) {
			loudnessMeter.setAveragingTime(oscValue);
		}
	});
	lufsGate.addChangeCallback([this](float oscValue) {
		for(auto& loudnessMeter : loudnessMeters) {
			loudnessMeter.setGateLevel(oscValue);
		}
	});
}

void CompressorFilter::init(size_t numChannel) {
	assert(this->numChannel == numChannel);
	// loudnessMeters.resize(numChannel);
}

void CompressorFilter::reset() {
	if(!enablePeak.get() && !enableLoudness.get()) {
		y1 = yL = gainComputer(0.f);
	}
}

void CompressorFilter::processSamples(float** samples, size_t count) {
	if(enablePeak || enableLoudness) {
		float staticGain = gainComputer(0) + makeUpGain;
		for(size_t i = 0; i < count; i++) {
			float levelPeak = 0;
			float levelLoudness = 0;

			float sampleLeft = samples[0][i];
			float sampleRight = samples[1][i];

			if(enablePeak.get()) {
				levelPeak = levelDetectorPeak(sampleLeft);
				levelPeak = std::max(levelPeak, levelDetectorPeak(sampleRight));
			}
			if(enableLoudness.get()) {
				levelLoudness = levelDetectorLoudnessLUFS(&loudnessMeters[0], sampleLeft);
				levelLoudness += levelDetectorLoudnessLUFS(&loudnessMeters[1], sampleRight);
			}

			float dbGain = doCompression(levelPeak, levelLoudness);

			// db to ratio
			float compressionRatio = fastpow2(LOG10_VALUE_DIV_20 * (dbGain + staticGain));

			for(size_t channel = 0; channel < numChannel; channel++) {
				samples[channel][i] = compressionRatio * samples[channel][i];
			}
		}
	}
}

float CompressorFilter::doCompression(float levelPeak, float levelLoudness) {
	// Compression with gain computer in the log domain
	// This is the log domain detector with feedforward design
	float dbSample = -INFINITY;

	if(enablePeak.get() && levelPeak != 0) {
		dbSample = levelToDbPeak(levelPeak);
	}
	if(enableLoudness.get() && levelLoudness != 0) {
		lufsRealtimeLevel = levelToDbLoudnessLUFS(levelLoudness);
		dbSample = std::max(dbSample, lufsRealtimeLevel - lufsTarget.get());
	}

	if(dbSample != -INFINITY) {
		levelDetectorSmoothing(gainComputer(dbSample));
	} else {
		// When a sample is 0, keep the current compression gain.
		levelDetectorSilenceSmoothing(gainComputer(0));
	}
	return -yL;
}

float CompressorFilter::levelDetectorPeak(float sample) {
	return fabsf(sample);
}

float CompressorFilter::levelDetectorLoudnessLUFS(LoudnessMeter* loudnessMeter, float sample) {
	loudnessMeter->processOneSample(sample);
	return loudnessMeter->getSquaredAverage();
}

float CompressorFilter::levelToDbPeak(float sample) {
	return fastlog2(sample) / LOG10_VALUE_DIV_20;
}

float CompressorFilter::levelToDbLoudnessLUFS(float sample) {
	return LoudnessMeter::linearToLufs(sample);
}

float CompressorFilter::gainComputer(float dbSample) const {
	float thresholdDistance = dbSample - threshold;
	float zone = 2 * thresholdDistance;
	if(zone <= -kneeWidth) {
		return 0;
	} else if(zone >= kneeWidth) {
		return gainDiffRatio * thresholdDistance;
	} else {
		float a = thresholdDistance + kneeWidth / 2;
		// Equivalent to gainDiffRatio * (a * a) / (2 * kneeWidth)
		return gainDiffRatioInKnee * (a * a);
	}
}

void CompressorFilter::levelDetectorSmoothing(float dbCompression) {
	// Smooth decoupled peak detector (eq 17)
	// Added moving max to try to reduce compression level changes
	float decayedCompression = alphaR * y1 + (1 - alphaR) * dbCompression;
	y1 = fmaxf(dbCompression, decayedCompression);
	yL = alphaA * yL + (1 - alphaA) * y1;
}

void CompressorFilter::levelDetectorSilenceSmoothing(float dbCompression) {
	// Smooth out to unity gain when the input signal is silence (sample value is 0).
	// Silence appears in these cases:
	//  1. When audio is stopped to switch to something else (that might have a different volume)
	//  2. When audio is silence temporarily (eg: radio/broadcast stream without background music)
	//  3. During normal audio, when the audio signal crosses 0
	//
	// In these cases, we want to:
	//  - Long term: tend to unity gain in case of switching to a high volume stream (handles 1.)
	//  - Short term: keep the current compression gain (handles 2. and 3.)
	// Use the release time as the constant time for this.
	y1 = alphaR * y1 + (1 - alphaR) * dbCompression;
	yL = alphaA * yL + (1 - alphaA) * y1;
}

void CompressorFilter::onFastTimer() {
	if(enableLoudness) {
		lufsMeter.set(lufsRealtimeLevel);
	} else {
		lufsMeter.set(0);
	}
}
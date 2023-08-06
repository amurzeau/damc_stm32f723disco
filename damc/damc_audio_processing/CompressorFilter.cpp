#include "CompressorFilter.h"

#include <MathUtils.h>
#include <algorithm>
#include <fastapprox/fastexp.h>
#include <fastapprox/fastlog.h>
#include <math.h>
#include <stm32f7xx.h>
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
	reset(48000);
	attackTime.addChangeCallback([this](float oscValue) { alphaA = oscValue != 0 ? expf(-1 / (oscValue * fs)) : 0; });
	releaseTime.addChangeCallback([this](float oscValue) { alphaR = oscValue != 0 ? expf(-1 / (oscValue * fs)) : 0; });
	ratio.addChangeCallback([this](float oscValue) { gainDiffRatio = 1 - 1 / oscValue; });
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
	this->numChannel = numChannel;
	loudnessMeters.resize(numChannel);
}

void CompressorFilter::reset(float fs) {
	this->fs = fs;
	y1 = yL = 0.f;
}

void CompressorFilter::processSamples(float** samples, size_t count) {
	if(enablePeak || enableLoudness) {
		float staticGain = gainComputer(0) + makeUpGain;
		for(size_t i = 0; i < count; i++) {
			float levelPeak = 0;
			float levelLoudness = 0;

			for(size_t channel = 0; channel < numChannel; channel++) {
				float sample = samples[channel][i];
				if(enablePeak.get()) {
					levelPeak = std::max(levelPeak, levelDetectorPeak(sample));
				}
				if(enableLoudness.get()) {
					levelLoudness += levelDetectorLoudnessLUFS(&loudnessMeters[channel], sample);
				}
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

	if(dbSample == -INFINITY) {
		return 0;
	}

	levelDetectorSmoothing(gainComputer(dbSample));
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
	float zone = 2 * (dbSample - threshold);
	if(zone == -INFINITY || zone <= -kneeWidth) {
		return 0;
	} else if(zone >= kneeWidth) {
		return gainDiffRatio * (dbSample - threshold);
	} else {
		float a = dbSample - threshold + kneeWidth / 2;
		return gainDiffRatio * (a * a) / (2 * kneeWidth);
	}
}

void CompressorFilter::levelDetectorSmoothing(float dbCompression) {
	// Smooth decoupled peak detector (eq 17)
	// Added moving max to try to reduce compression level changes
	float decayedCompression = alphaR * y1 + (1 - alphaR) * dbCompression;
	y1 = fmaxf(dbCompression, decayedCompression);
	yL = alphaA * yL + (1 - alphaA) * y1;
}

void CompressorFilter::onFastTimer() {
	if(enableLoudness) {
		lufsMeter.set(lufsRealtimeLevel);
	} else {
		lufsMeter.set(0);
	}
}
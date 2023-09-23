#pragma once

#include "LoudnessMeter.h"
#include <Osc/OscContainer.h>
#include <Osc/OscReadOnlyVariable.h>
#include <Osc/OscVariable.h>
#include <array>
#include <deque>
#include <stddef.h>

class CompressorFilter : public OscContainer {
protected:
public:
	CompressorFilter(OscContainer* parent);
	void init(size_t numChannel);
	void reset(float fs);
	void processSamples(float** samples, size_t count);

	void onFastTimer();

protected:
	float doCompression(float levelPeak, float levelLoudness);
	float gainComputer(float sample) const;
	void levelDetectorSmoothing(float sample);
	float levelToDbPeak(float sample);
	float levelToDbLoudnessLUFS(float sample);
	float levelDetectorPeak(float sample);
	float levelDetectorLoudnessLUFS(LoudnessMeter* loudnessMeter, float sample);

private:
	size_t numChannel;

	// Level to dB function to use
	using LevelToDbFunction = float (CompressorFilter::*)(float sample);
	LevelToDbFunction levelDetector;
	LevelToDbFunction levelToDb;
	// Level detector state
	float y1;
	float yL;

	// Loudness LUFS level detector
	std::array<LoudnessMeter, 2> loudnessMeters;

	OscVariable<bool> enablePeak;
	OscVariable<bool> enableLoudness;
	float fs;
	float alphaR;
	float alphaA;
	OscVariable<float> attackTime;
	OscVariable<float> releaseTime;
	OscVariable<float> threshold;
	OscVariable<float> makeUpGain;
	OscVariable<float> ratio;
	float gainDiffRatio = 0;
	OscVariable<float> kneeWidth;
	OscVariable<float> lufsTarget;
	OscVariable<float> lufsIntegrationTime;
	OscVariable<float> lufsGate;

	float lufsRealtimeLevel;
	OscReadOnlyVariable<float> lufsMeter;
};

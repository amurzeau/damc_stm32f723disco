#pragma once

#include <LoudnessMeter.h>
#include <Osc/OscVariable.h>
#include <Osc/OscDynamicVariable.h>
// #include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

class OscContainer;

class ControlInterface;

class PeakMeter {
public:
	PeakMeter(OscContainer* parent,
	          OscReadOnlyVariable<int32_t>* oscNumChannel,
	          OscReadOnlyVariable<int32_t>* oscSampleRate);
	~PeakMeter();

	void processSamples(const float* peaks, size_t numChannels, size_t samplesInPeaks);

	void onFastTimer();

	// std::vector<LoudnessMeter> loudnessMeters;

private:
	OscRoot* oscRoot;
	OscReadOnlyVariable<int32_t>* oscSampleRate;
	OscDynamicVariable<float> oscPeakGlobal;
	OscDynamicVariable<float> oscPeakPerChannel;

	std::vector<float> levelsDb;
	// std::mutex peakMutex;
	int samplesInPeaks;
	std::array<float, 2> peaksPerChannel;
	std::array<float, 2> peaksPerChannelToSend;
	std::string oscPeakGlobalPath;
	std::string oscPeakPerChannelPath;
	std::vector<OscArgument> oscPeakPerChannelArguments;

	OscVariable<bool> oscEnablePeakUpdate;
};

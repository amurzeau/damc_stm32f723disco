#include "CompressorFilter.h"

#include <algorithm>
#include <math.h>
#include <string.h>
#include <fastapprox/fastexp.h>
#include <fastapprox/fastlog.h>

const float CompressorFilter::LOG10_VALUE_DIV_20 = fastlog2(10) / 20;

CompressorFilter::CompressorFilter(OscContainer* parent)
    : OscContainer(parent, "compressorFilter"),
      enable(this, "enable", false),
      attackTime(this, "attackTime", 0),
      releaseTime(this, "releaseTime", 2),
      threshold(this, "threshold", -50),
      makeUpGain(this, "makeUpGain", 0),
      ratio(this, "ratio", 1000),
      kneeWidth(this, "kneeWidth", 0),
      useMovingMax(this, "useMovingMax", false) {
	attackTime.addChangeCallback([this](float oscValue) { alphaA = oscValue != 0 ? expf(-1 / (oscValue * fs)) : 0; });
	releaseTime.addChangeCallback([this](float oscValue) { alphaR = oscValue != 0 ? expf(-1 / (oscValue * fs)) : 0; });
	ratio.addChangeCallback([this](float oscValue) { gainDiffRatio = 1 - 1 / oscValue; });
}

void CompressorFilter::init(size_t numChannel) {
	this->numChannel = numChannel;
	perChannelData.resize(numChannel);
}

void CompressorFilter::reset(double fs) {
	this->fs = fs;
	gainHoldSamples = (uint32_t)(fs / 20);
	std::fill_n(perChannelData.begin(), numChannel, PerChannelData{});
}

void CompressorFilter::processSamples(float** samples, size_t count) {
	if(enable) {
		float staticGain = gainComputer(0) + makeUpGain;
		for(size_t i = 0; i < count; i++) {
			float largerCompressionDb = 0;
			for(size_t channel = 0; channel < numChannel; channel++) {
				float dbGain = doCompression(samples[channel][i], perChannelData[channel]);
				if(dbGain < largerCompressionDb)
					largerCompressionDb = dbGain;
			}

			// db to ratio
			float largerCompressionRatio = fastpow2(LOG10_VALUE_DIV_20 * (largerCompressionDb + staticGain));

			for(size_t channel = 0; channel < numChannel; channel++) {
				samples[channel][i] = largerCompressionRatio * samples[channel][i];
			}
		}
	}
}

float CompressorFilter::doCompression(float sample, PerChannelData& perChannelData) {
	if(sample == 0)
		return 0;

	float dbSample = fastlog2(fabsf(sample)) / LOG10_VALUE_DIV_20;
	levelDetector(gainComputer(dbSample), perChannelData);
	return -perChannelData.yL;
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

float CompressorFilter::PerChannelData::movingMax(float dbCompression) {
	if(!compressionMovingMaxDeque.empty() && compressionMovingMaxDeque.front() == compressionHistoryPtr)
		compressionMovingMaxDeque.pop_front();

	while(!compressionMovingMaxDeque.empty() && dbCompression >= compressionHistory[compressionMovingMaxDeque.back()])
		compressionMovingMaxDeque.pop_back();

	compressionMovingMaxDeque.push_back(compressionHistoryPtr);
	compressionHistory[compressionHistoryPtr] = dbCompression;
	compressionHistoryPtr = (compressionHistoryPtr + 1) % compressionHistory.size();

	return compressionHistory[compressionMovingMaxDeque.front()];
}

float CompressorFilter::PerChannelData::noProcessing(float dbCompression) {
	return dbCompression;
}

void CompressorFilter::levelDetector(float dbCompression, PerChannelData& perChannelData) {
	float decayedCompression = alphaR * perChannelData.y1 + (1 - alphaR) * dbCompression;
	if(useMovingMax)
		perChannelData.y1 = fmaxf(perChannelData.movingMax(dbCompression), decayedCompression);
	else
		perChannelData.y1 = fmaxf(dbCompression, decayedCompression);
	perChannelData.yL = alphaA * perChannelData.yL + (1 - alphaA) * perChannelData.y1;
}

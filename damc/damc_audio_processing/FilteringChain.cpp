#include "FilteringChain.h"
#include <MathUtils.h>
#include <Utils.h>
#include <fastapprox/fastexp.h>
#include <fastapprox/fastlog.h>
#include <math.h>
#include <string.h>

float LogScaleFromOsc(float value) {
	return fastpow2(value * LOG10_VALUE_DIV_20);
}

float LogScaleToOsc(float value) {
	float logValue = fastlog2(value) / LOG10_VALUE_DIV_20;
	logValue = roundf(logValue * 100.f) / 100.f;
	return logValue;
}

FilterChain::FilterChain(OscContainer* parent,
                         OscReadOnlyVariable<int32_t>* oscNumChannel,
                         OscReadOnlyVariable<int32_t>* oscSampleRate)
    : OscContainer(parent, "filterChain", 10),
      // reverbFilters(this, "reverbFilter"),
      oscEqFilters(this, "eqFilters", &eqFilters),
      eqFilters{
          EqFilter{&oscEqFilters, Utils::toString(0)},
          EqFilter{&oscEqFilters, Utils::toString(1)},
          EqFilter{&oscEqFilters, Utils::toString(2)},
          EqFilter{&oscEqFilters, Utils::toString(3)},
          EqFilter{&oscEqFilters, Utils::toString(4)},
          EqFilter{&oscEqFilters, Utils::toString(5)},
          EqFilter{&oscEqFilters, Utils::toString(6)},
          EqFilter{&oscEqFilters, Utils::toString(7)},
          EqFilter{&oscEqFilters, Utils::toString(8)},
          EqFilter{&oscEqFilters, Utils::toString(9)},
      },
      compressorFilter(this),
      expanderFilter(this),
      peakMeter(parent, oscNumChannel, oscSampleRate),
      delay(this, "delay", 0),
      oscVolume(this, "balance", 1.0f),
      volume{1.0f, 1.0f},
      masterVolume(this, "volume", 1.0f),
      mute(this, "mute", false),
      reverseAudioSignal(this, "reverseAudioSignal", false) {
	delay.addChangeCallback([this](int32_t newValue) {
		for(DelayFilter& filter : delayFilters) {
			filter.setParameters(newValue);
		}
	});
	oscVolume.setOscConverters(&LogScaleToOsc, &LogScaleFromOsc);
	masterVolume.setOscConverters(&LogScaleToOsc, &LogScaleFromOsc);

	oscNumChannel->addChangeCallback([this](int32_t newValue) {
		if(newValue > 0)
			updateNumChannels(newValue);
	});
}

void FilterChain::updateNumChannels(size_t numChannel) {
	// reverbFilters.resize(numChannel);
	oscVolume.resize(numChannel);

	for(auto& filter : eqFilters) {
		filter.init(numChannel);
	}

	compressorFilter.init(numChannel);
	expanderFilter.init(numChannel);
}

void FilterChain::reset(float fs) {
	for(DelayFilter& delayFilter : delayFilters) {
		delayFilter.reset();
	}
	//	for(auto& reverbFilter : reverbFilters) {
	//		reverbFilter.second->reset();
	//	}

	for(auto& filter : eqFilters) {
		filter.reset(fs);
	}

	compressorFilter.reset(fs);
	expanderFilter.reset(fs);
}

void FilterChain::processSamples(float** samples, size_t numChannel, size_t count) {
	float* peaks = (float*) alloca(sizeof(float) * numChannel);
	float masterVolume = this->masterVolume.get();

	if(reverseAudioSignal) {
		masterVolume *= -1;
	}

	for(uint32_t channel = 0; channel < numChannel; channel++) {
		delayFilters[channel].processSamples(samples[channel], count);
	}

	for(auto& filter : eqFilters) {
		filter.processSamples(samples, count);
	}

	expanderFilter.processSamples(samples, count);
	compressorFilter.processSamples(samples, count);

	//	for(uint32_t channel = 0; channel < numChannel; channel++) {
	//		reverbFilters.at(channel).processSamples(samples[channel], count);
	//	}

	for(uint32_t channel = 0; channel < numChannel; channel++) {
		float volume = this->volume[channel] * masterVolume;
		float peak = 0;
		for(size_t i = 0; i < count; i++) {
			samples[channel][i] *= volume;
			peak = fmaxf(peak, fabsf(samples[channel][i]));
		}
		peaks[channel] = peak;
	}

	// for(uint32_t channel = 0; channel < numChannel; channel++) {
	// 	peakMeter.loudnessMeters[channel].processSamples(samples[channel], count);
	// }
	peakMeter.processSamples(peaks, numChannel, count);

	if(mute) {
		for(uint32_t channel = 0; channel < numChannel; channel++) {
			std::fill_n(samples[channel], count, 0);
		}
	}
}

float FilterChain::processSideChannelSample(float input) {
	return delayFilters.back().processOneSample(input);
}

void FilterChain::onFastTimer() {
	peakMeter.onFastTimer();
	compressorFilter.onFastTimer();

	for(size_t i = 0; i < volume.size(); i++) {
		volume[i] = oscVolume.at(i).get();
	}
}

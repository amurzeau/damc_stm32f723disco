#include "FilteringChain.h"
#include <MathUtils.h>
#include <fastapprox/fastexp.h>
#include <fastapprox/fastlog.h>
#include <math.h>
#include <string.h>
#include <Utils.h>

float LogScaleFromOsc(float value) {
	return fastpow2(value * LOG10_VALUE_DIV_20);
}

float LogScaleToOsc(float value) {
	return fastlog2(value) / LOG10_VALUE_DIV_20;
}

FilterChain::FilterChain(OscContainer* parent,
                         OscReadOnlyVariable<int32_t>* oscNumChannel,
                         OscReadOnlyVariable<int32_t>* oscSampleRate)
    : OscContainer(parent, "filterChain"),
      // reverbFilters(this, "reverbFilter"),
      eqFilters(this, "eqFilters"),
      compressorFilter(this),
      expanderFilter(this),
      peakMeter(parent, oscNumChannel, oscSampleRate),
      delay(this, "delay", 0),
      volume(this, "balance", 1.0f),
      masterVolume(this, "volume", 1.0f),
      mute(this, "mute", false),
      reverseAudioSignal(this, "reverseAudioSignal", false) {
	//	reverbFilters.setFactory(
	//	    [](OscContainer* parent, int name) { return new ReverbFilter(parent, Utils::toString(name)); });
	eqFilters.setFactory([](OscContainer* parent, int name) { return new EqFilter(parent, Utils::toString(name)); });

	delay.addChangeCallback([this](int32_t newValue) {
		for(DelayFilter& filter : delayFilters) {
			filter.setParameters(newValue);
		}
	});
	volume.setOscConverters(&LogScaleToOsc, &LogScaleFromOsc);
	masterVolume.setOscConverters(&LogScaleToOsc, &LogScaleFromOsc);

	oscNumChannel->addChangeCallback([this](int32_t newValue) {
		if(newValue > 0)
			updateNumChannels(newValue);
	});
}

void FilterChain::updateNumChannels(size_t numChannel) {
	delayFilters.resize(numChannel + 1);  // +1 for side channel
	// reverbFilters.resize(numChannel);
	volume.resize(numChannel);

	eqFilters.resize(6);

	for(auto& filter : eqFilters) {
		filter.second->init(numChannel);
	}

	compressorFilter.init(numChannel);
	expanderFilter.init(numChannel);
}

void FilterChain::reset(double fs) {
	for(DelayFilter& delayFilter : delayFilters) {
		delayFilter.reset();
	}
	//	for(auto& reverbFilter : reverbFilters) {
	//		reverbFilter.second->reset();
	//	}

	for(auto& filter : eqFilters) {
		filter.second->reset(fs);
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
		filter.second->processSamples(samples, count);
	}

	expanderFilter.processSamples(samples, count);
	compressorFilter.processSamples(samples, count);

	//	for(uint32_t channel = 0; channel < numChannel; channel++) {
	//		reverbFilters.at(channel).processSamples(samples[channel], count);
	//	}

	for(uint32_t channel = 0; channel < numChannel; channel++) {
		float volume = this->volume.at(channel).get() * masterVolume;
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
}

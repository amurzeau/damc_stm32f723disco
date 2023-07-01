#include "AudioProcessor.h"
#include "OscSerialClient.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "TimeMeasure.h"

#include <spdlog/spdlog.h>

#include <stm32f7xx_hal.h>
#include <usbd_conf.h>


AudioProcessor* AudioProcessor::getInstance() {
	static AudioProcessor instance(2, 48000, 48);
	return &instance;
}

volatile uint32_t variable = sizeof(ChannelStrip);

AudioProcessor::AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes)
    : oscRoot(true), serialClient(&oscRoot), strips(&oscRoot, "strip"),
	  timeMeasureUsbInterrupt(&oscRoot, "timeUsbInterrupt"),
	  timeMeasureAudioProcessing(&oscRoot, "timeAudioProc"),
	  timeMeasureFastTimer(&oscRoot, "timeFastTimer"),
	  timeMeasureOscInput(&oscRoot, "timeOscInput"),
	  memoryAvailable(&oscRoot, "memoryAvailable"),
	  memoryUsed(&oscRoot, "memoryUsed")
{
	strips.setFactory([this, numChannels, sampleRate, maxNframes](OscContainer* parent, int name) {
		return new ChannelStrip(parent, name, numChannels, sampleRate, maxNframes);
	});

	strips.resize(AUDIO_LOOPBACKS_NUMBER);

	serialClient.init();
}

AudioProcessor::~AudioProcessor() {}

void AudioProcessor::processAudioInterleaved(int16_t index,
                                             const int16_t* data_input,
                                             int16_t* data_output,
                                             size_t nframes) {
	TimeMeasure::timeMeasureAudioProcessing.beginMeasure();

	if(index < AUDIO_LOOPBACKS_NUMBER)
		strips.at(index).processAudioInterleaved(data_input, data_output, nframes);
	else {
		memcpy(data_output, data_input, nframes * USBD_AUDIO_BYTES_PER_SAMPLE * USBD_AUDIO_CHANNELS);
	}

	TimeMeasure::timeMeasureAudioProcessing.endMeasure();
}

extern "C" uint8_t *__sbrk_heap_end;
extern "C" uint8_t _end;
extern "C" uint8_t _estack;
extern "C" uint8_t _Min_Stack_Size;
void AudioProcessor::mainLoop() {
	serialClient.mainLoop();

	uint32_t currentTick = HAL_GetTick();
	// Do onFastTimer every 100ms
	// Process one strip at a time to avoid taking too much time
	if(currentTick >= fastTimerPreviousTick + (100/AUDIO_LOOPBACKS_NUMBER)) {
		TimeMeasure::timeMeasureFastTimer.beginMeasure();
		fastTimerPreviousTick = currentTick;
		strips.at(nextTimerStripIndex).onFastTimer();
		nextTimerStripIndex++;
		if(nextTimerStripIndex >= AUDIO_LOOPBACKS_NUMBER)
			nextTimerStripIndex = 0;
		TimeMeasure::timeMeasureFastTimer.endMeasure();
	}

	currentTick = HAL_GetTick();
	if(currentTick >= slowTimerPreviousTick + 1000) {
		slowTimerPreviousTick = currentTick;
		timeMeasureUsbInterrupt.set(TimeMeasure::timeMeasureUsbInterrupt.getCumulatedTimeUsAndReset());
		timeMeasureAudioProcessing.set(TimeMeasure::timeMeasureAudioProcessing.getCumulatedTimeUsAndReset());
		timeMeasureFastTimer.set(TimeMeasure::timeMeasureFastTimer.getCumulatedTimeUsAndReset());
		timeMeasureOscInput.set(TimeMeasure::timeMeasureOscInput.getCumulatedTimeUsAndReset());

		OscArgument used_memory = static_cast<int32_t>((uint32_t)__sbrk_heap_end - (uint32_t)&_end);
		OscArgument available_memory = static_cast<int32_t>((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size - (uint32_t)__sbrk_heap_end);
		memoryUsed.sendMessage(&used_memory, 1);
		memoryAvailable.sendMessage(&available_memory, 1);
	}
}

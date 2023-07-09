#include "AudioProcessor.h"
#include "OscSerialClient.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "TimeMeasure.h"
#include <CodecAudio.h>
#include <vector>
#include <map>

#include <spdlog/spdlog.h>

#include <stm32f7xx_hal.h>
#include <usbd_conf.h>


volatile AudioProcessor* audio_processor;

MultiChannelAudioBuffer::MultiChannelAudioBuffer() {
	for(size_t i = 0; i < CHANNEL_NUMBER; i++) {
		dataPointers[i] = data[i];
	}
}

AudioProcessor* AudioProcessor::getInstance() {
	static AudioProcessor instance(2, 48000, 48);
	audio_processor = &instance;
	return &instance;
}

AudioProcessor::AudioProcessor(uint32_t numChannels, uint32_t sampleRate, size_t maxNframes)
    : numChannels(numChannels),
	  oscRoot(true),
	  serialClient(&oscRoot),
	  strips(&oscRoot, "strip"),
	  timeMeasureUsbInterrupt(&oscRoot, "timeUsbInterrupt"),
	  timeMeasureAudioProcessing(&oscRoot, "timeAudioProc"),
	  timeMeasureFastTimer(&oscRoot, "timeFastTimer"),
	  timeMeasureOscInput(&oscRoot, "timeOscInput"),
	  timeMeasureMaxPerLoopUsbInterrupt(&oscRoot, "timePerLoopUsbInterrupt"),
	  timeMeasureMaxPerLoopAudioProcessing(&oscRoot, "timePerLoopAudioProc"),
	  timeMeasureMaxPerLoopFastTimer(&oscRoot, "timePerLoopFastTimer"),
	  timeMeasureMaxPerLoopOscInput(&oscRoot, "timePerLoopOscInput"),
	  memoryAvailable(&oscRoot, "memoryAvailable"),
	  memoryUsed(&oscRoot, "memoryUsed"),
	  fastTimerPreviousTick(0),
	  nextTimerStripIndex(0),
	  slowTimerPreviousTick(0),
	  slowTimerIndex(0)
{
	strips.setFactory([this, numChannels, sampleRate, maxNframes](OscContainer* parent, int index) {
		using namespace std::literals;

		std::string_view name;
		switch(index) {
		case 0:
			name = "master"sv;
			break;
		case 1:
			name = "comp"sv;
			break;
		case 2:
			name = "mic"sv;
			break;
		case 3:
			name = "out-record"sv;
			break;
		case 4:
			name = "mic-feedback"sv;
			break;
		default:
			name = Utils::toString(index);
			break;
		}
		return new ChannelStrip(parent, index, name, numChannels, sampleRate, maxNframes);
	});

	strips.resize(5);

	serialClient.init();
}

AudioProcessor::~AudioProcessor() {}

void AudioProcessor::init() {
	using namespace std::literals;
	const std::map<std::string_view, std::vector<OscArgument>> default_config = {
		{"/strip/1/filterChain/compressorFilter/makeUpGain", {float{-1.f}}},
		{"/strip/0/display_name", {"master"sv}},
		{"/strip/1/display_name", {"comp"sv}},
		{"/strip/2/display_name", {"mic"sv}},
		{"/strip/3/display_name", {"out-record"sv}},
		{"/strip/4/display_name", {"mic-feedback"sv}},
		{"/strip/1/filterChain/compressorFilter/enable", {true}},
		{"/strip/3/filterChain/mute", {true}},
		{"/strip/4/filterChain/mute", {true}},
	};

	oscRoot.loadNodeConfig(default_config);
}

void AudioProcessor::interleavedToFloat(const int16_t* data_input, MultiChannelAudioBuffer* data_float, size_t nframes) {
	for(uint32_t channel = 0; channel < numChannels; channel++) {
		for(uint32_t frame = 0; frame < nframes; frame++) {
			data_float->data[channel][frame] = data_input[frame * numChannels + channel] / 32768.f;
		}
	}
}

void AudioProcessor::floatToInterleaved(MultiChannelAudioBuffer* data_float, int16_t* data_output, size_t nframes) {
	for(uint32_t channel = 0; channel < numChannels; channel++) {
		for(uint32_t frame = 0; frame < nframes; frame++) {
			data_output[frame * numChannels + channel] =
			    static_cast<int16_t>(data_float->data[channel][frame] * 32768.f);
		}
	}
}

void AudioProcessor::mixAudio(MultiChannelAudioBuffer* mixed_data, MultiChannelAudioBuffer* data_to_add, size_t nframes) {
	for(uint32_t channel = 0; channel < numChannels; channel++) {
		for(uint32_t frame = 0; frame < nframes; frame++) {
			mixed_data->data[channel][frame] += data_to_add->data[channel][frame];
		}
	}
}

void AudioProcessor::processAudioInterleaved(
		const int16_t** input_endpoints,
		size_t input_endpoints_number,
		int16_t** output_endpoints,
		size_t output_endpoints_number,
		size_t nframes) {
	TimeMeasure::timeMeasureAudioProcessing.beginMeasure();
	/**
	 * Legend:
	 *  - OUT/IN: USB endpoints (OUT = from PC to codec, IN = from codec to PC)
	 *  - (+): audio mixer (add all inputs)
	 *  - [N]: ChannelStrip N that process audio (N starts at 0)
	 *  - *: connection, audio goes to multiple destinations
	 *  - +: signal line corner (purely graphical)
	 *
	 * OUT 0 (uncompressed output)    --------->(+)--->[0]--*---(+)---> Codec Headphones
	 * OUT 1 (audio to be compressed) --> [1] ---^          |    |
	 *                                                      |    |
	 * IN 0 (mic input)               <----------(+)---[3]--+   [4]
	 *                                            |              |
	 *                                            +----[2]-------*----< Codec mic
	 * ChannelStrip:
	 *   [0]: master (OUT 0 + compressed OUT 1)
	 *   [1]: compressor for OUT 1
	 *   [2]: mic
	 *   [3]: output record loopback
	 *   [4]: mic feedback
	 *
	 * Intermediate variables:
	 *  - #0: OUT 1 processing
	 *  - #1: OUT 0 processing then mix of OUT 0 + OUT 1 then out-record processing then IN 0
	 *  - #2: Codec Headphones mix
	 *  - #3: Codec MIC input then MIC processing (then added into buffer #1)
	 *  - #4: Copy of codec MIC input then mic-feedback processing (then added into buffer #2)
	 *
	 */

	// Import endpoint OUT 1 (compressed audio) into float
	interleavedToFloat(input_endpoints[1], &buffer[0], nframes);

	// Process comp
	strips.at(1).processSamples(buffer[0].dataPointers, numChannels, nframes);

	// Import endpoint OUT 0 into float
	interleavedToFloat(input_endpoints[0], &buffer[1], nframes);

	// Mix OUT 0 with compressed (by strip 1) OUT 1
	mixAudio(&buffer[1], &buffer[0], nframes);

	// Process master
	strips.at(0).processSamples(buffer[1].dataPointers, numChannels, nframes);

	// Copy output as it has multiple destination
	memcpy(&buffer[2].data, &buffer[1].data, sizeof(buffer[1].data));

	// Process out-record for IN 0
	strips.at(3).processSamples(buffer[1].dataPointers, numChannels, nframes);

	// Get codec MIC data
	CodecAudio::instance.processAudioInterleavedInput(codecBuffer, nframes);
	interleavedToFloat(codecBuffer, &buffer[3], nframes);

	// Copy codec MIC audio as it has multiple destination
	memcpy(&buffer[4].data, &buffer[3].data, sizeof(buffer[3].data));


	// Process mic
	strips.at(2).processSamples(buffer[3].dataPointers, numChannels, nframes);

	// Mic mic and out-record into IN 0
	mixAudio(&buffer[1], &buffer[3], nframes);

	// Output float data to USB endpoint IN 0
	floatToInterleaved(&buffer[1], output_endpoints[0], nframes);

	// Process mic-feedback
	strips.at(4).processSamples(buffer[4].dataPointers, numChannels, nframes);

	// Mix mic-feedback with master
	mixAudio(&buffer[2], &buffer[4], nframes);

	// Output float data to codec headphones
	floatToInterleaved(&buffer[2], codecBuffer, nframes);

	// Output processed audio to codec and retrieve MIC
	CodecAudio::instance.processAudioInterleavedOutput(codecBuffer, nframes);

	TimeMeasure::timeMeasureAudioProcessing.endMeasure();
}

extern "C" uint8_t *__sbrk_heap_end; // end of heap
extern "C" uint8_t _end; // end of static data in RAM (start of heap)
extern "C" uint8_t _estack; // start of RAM (end of RAM as stack grows backward)
extern "C" uint8_t _Min_Stack_Size; // minimal stack size
void AudioProcessor::mainLoop() {
	// Do at most one thing to avoid taking too much time here
	if(serialClient.mainLoop())
		return;

	uint32_t currentTick = HAL_GetTick();
	// Do onFastTimer every 100ms
	// Process one strip at a time to avoid taking too much time
	if(currentTick >= fastTimerPreviousTick + (100/strips.size())) {
		TimeMeasure::timeMeasureFastTimer.beginMeasure();

		fastTimerPreviousTick = currentTick;
		strips.at(nextTimerStripIndex).onFastTimer();
		nextTimerStripIndex++;
		if(nextTimerStripIndex >= strips.size())
			nextTimerStripIndex = 0;

		TimeMeasure::timeMeasureFastTimer.endMeasure();
	} else if(currentTick >= slowTimerPreviousTick + 1000/3) {
		TimeMeasure::timeMeasureFastTimer.beginMeasure();

		slowTimerPreviousTick = currentTick;

		switch(slowTimerIndex) {
		case 0:
			timeMeasureUsbInterrupt.set(TimeMeasure::timeMeasureUsbInterrupt.getCumulatedTimeUsAndReset());
			timeMeasureAudioProcessing.set(TimeMeasure::timeMeasureAudioProcessing.getCumulatedTimeUsAndReset());
			timeMeasureFastTimer.set(TimeMeasure::timeMeasureFastTimer.getCumulatedTimeUsAndReset());
			timeMeasureOscInput.set(TimeMeasure::timeMeasureOscInput.getCumulatedTimeUsAndReset());
			break;
		case 1:
			timeMeasureMaxPerLoopUsbInterrupt.set(TimeMeasure::timeMeasureUsbInterrupt.getMaxTimeUsAndReset());
			timeMeasureMaxPerLoopAudioProcessing.set(TimeMeasure::timeMeasureAudioProcessing.getMaxTimeUsAndReset());
			timeMeasureMaxPerLoopFastTimer.set(TimeMeasure::timeMeasureFastTimer.getMaxTimeUsAndReset());
			timeMeasureMaxPerLoopOscInput.set(TimeMeasure::timeMeasureOscInput.getMaxTimeUsAndReset());
			break;
		case 2:
			OscArgument used_memory = static_cast<int32_t>((uint32_t)__sbrk_heap_end - (uint32_t)&_end);
			memoryUsed.sendMessage(&used_memory, 1);

			OscArgument available_memory = static_cast<int32_t>((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size - (uint32_t)__sbrk_heap_end);
			memoryAvailable.sendMessage(&available_memory, 1);
			break;
		}
		slowTimerIndex++;
		if(slowTimerIndex > 2)
			slowTimerIndex = 0;

		TimeMeasure::timeMeasureFastTimer.endMeasure();
	}

	TimeMeasure::timeMeasureUsbInterrupt.endAudioLoop();
	TimeMeasure::timeMeasureAudioProcessing.endAudioLoop();
	TimeMeasure::timeMeasureFastTimer.endAudioLoop();
	TimeMeasure::timeMeasureOscInput.endAudioLoop();
}

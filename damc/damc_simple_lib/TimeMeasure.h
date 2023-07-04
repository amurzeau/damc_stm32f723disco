#pragma once

#include <stdint.h>

class TimeMeasure {
public:
	TimeMeasure();

	void beginMeasure();
	void endMeasure();
	void endAudioLoop();

	static uint32_t getCurrent();

	uint32_t getCumulatedTimeUsAndReset();
	uint32_t getMaxTimeUsAndReset();

	static void updateClockPerUs();

	static TimeMeasure timeMeasureUsbInterrupt;
	static TimeMeasure timeMeasureAudioProcessing;
	static TimeMeasure timeMeasureFastTimer;
	static TimeMeasure timeMeasureOscInput;

private:
	static uint32_t clock_per_us;
	uint64_t time_sum;
	uint64_t time_sum_per_loop;
	uint32_t time_max;
	uint32_t begin_time;
};

#pragma once

#include <AudioCApi.h>
#include <stdint.h>

// Order by priority

class TimeMeasure {
public:
	TimeMeasure();

	void beginMeasure();
	void endMeasure();
	void endAudioLoop();

	/** @brief Return TIM2 counter in microseconds.
	 */
	static uint32_t getCurrent();

	uint32_t getCumulatedTimeUsAndReset();
	uint32_t getMaxTimeUsAndReset();
	uint32_t getMaxTimeUs() { return time_max; }
	uint32_t getOnGoingDuration();

	static void on1msElapsed();

	static TimeMeasure timeMeasure[TMI_NUMBER];

protected:
	bool updateMeasureAndStop(uint32_t current_time);

private:
	uint32_t index;
	uint32_t current_measure_sum;
	uint32_t time_sum_between_reset;
	uint32_t time_sum;
	uint32_t time_sum_per_loop;
	uint32_t time_max_between_reset;
	uint32_t time_max;
	uint32_t begin_time;
	bool isMeasuring;

	bool otherTimeMeasureState[TMI_NUMBER];
};

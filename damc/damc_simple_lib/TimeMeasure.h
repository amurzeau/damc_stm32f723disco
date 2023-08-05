#pragma once

#include <stdint.h>

enum TimeMeasureItem {
	TMI_UsbInterrupt,
	TMI_AudioProcessing,
	TMI_MainLoop,
	TMI_OscInput,

	TMI_NUMBER,
};

class TimeMeasure {
public:
	TimeMeasure();

	void beginMeasure(bool stateSave = true);
	void endMeasure(bool stateRestore = true);
	void endAudioLoop();

	static uint32_t getCurrent();

	uint32_t getCumulatedTimeUsAndReset();
	uint32_t getMaxTimeUsAndReset();

	static void updateClockPerUs();
	static void on1msElapsed();

	static TimeMeasure timeMeasure[TMI_NUMBER];

private:
	static uint32_t clock_per_us;
	uint64_t time_sum;
	uint64_t time_sum_per_loop;
	uint32_t time_max;
	uint32_t begin_time;
	bool isMeasuring;

	bool otherTimeMeasureState[TMI_NUMBER];
};

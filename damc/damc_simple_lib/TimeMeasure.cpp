#include "TimeMeasure.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_rcc.h>
#include <string.h>

uint32_t TimeMeasure::clock_per_us = 1;
TimeMeasure TimeMeasure::timeMeasure[TMI_NUMBER];

void TimeMeasure::updateClockPerUs() {
	clock_per_us = 108;
}

void TimeMeasure::on1msElapsed() {
	for(size_t i = 0; i < TMI_NUMBER; i++) {
		timeMeasure[i].endAudioLoop();
	}
}

TimeMeasure::TimeMeasure() : time_sum(0), time_sum_per_loop(0), time_max(0), begin_time(0), isMeasuring(false) {
	memset(otherTimeMeasureState, 0, sizeof(otherTimeMeasureState));
}

void TimeMeasure::beginMeasure(bool stateSave) {
	// Pause others
	if(stateSave) {
		for(size_t i = 0; i < TMI_NUMBER; i++) {
			if(&timeMeasure[i] == this)
				continue;
			otherTimeMeasureState[i] = timeMeasure[i].isMeasuring;
			if(otherTimeMeasureState[i]) {
				timeMeasure[i].endMeasure(false);
			}
		}
	}

	begin_time = TIM2->CNT;
	__DSB();
	isMeasuring = true;
}
void TimeMeasure::endMeasure(bool stateRestore) {
	isMeasuring = false;
	__DSB();

	uint32_t time_measured = TIM2->CNT - begin_time;
	__disable_irq();
	time_sum += time_measured;
	time_sum_per_loop += time_measured;
	__enable_irq();

	// Restart paused measures
	if(stateRestore) {
		for(size_t i = 0; i < TMI_NUMBER; i++) {
			if(&timeMeasure[i] == this)
				continue;
			if(otherTimeMeasureState[i]) {
				timeMeasure[i].beginMeasure(false);
			}
			otherTimeMeasureState[i] = false;
		}
	}
}

void TimeMeasure::endAudioLoop() {
	__disable_irq();
	if(time_sum_per_loop > time_max)
		time_max = time_sum_per_loop;
	time_sum_per_loop = 0;
	__enable_irq();
}

uint32_t TimeMeasure::getCurrent() {
	return TIM2->CNT;
}

uint32_t TimeMeasure::getCumulatedTimeUsAndReset() {
	uint64_t measured_time_us = time_sum / clock_per_us;
	time_sum = 0;
	return measured_time_us;
}

uint32_t TimeMeasure::getMaxTimeUsAndReset() {
	uint64_t measured_time_us = time_max / clock_per_us;
	time_max = 0;
	return measured_time_us;
}

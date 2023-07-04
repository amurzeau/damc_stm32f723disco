#include "TimeMeasure.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_rcc.h>

uint32_t TimeMeasure::clock_per_us = 1;
TimeMeasure TimeMeasure::timeMeasureUsbInterrupt;
TimeMeasure TimeMeasure::timeMeasureAudioProcessing;
TimeMeasure TimeMeasure::timeMeasureFastTimer;
TimeMeasure TimeMeasure::timeMeasureOscInput;


void TimeMeasure::updateClockPerUs() {
	clock_per_us = 108;
}

TimeMeasure::TimeMeasure() : time_sum(0), time_sum_per_loop(0), time_max(0), begin_time(0) {
}

void TimeMeasure::beginMeasure() {
	begin_time = TIM2->CNT;
}
void TimeMeasure::endMeasure() {
	uint32_t time_measured = TIM2->CNT - begin_time;
	time_sum += time_measured;
	time_sum_per_loop += time_measured;
}

void TimeMeasure::endAudioLoop() {
	if(time_sum_per_loop > time_max)
		time_max = time_sum_per_loop;
	time_sum_per_loop = 0;
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

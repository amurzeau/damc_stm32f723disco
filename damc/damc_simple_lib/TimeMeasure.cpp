#include "TimeMeasure.h"
#include <stm32f7xx.h>
#include <string.h>

TimeMeasure TimeMeasure::timeMeasure[TMI_NUMBER];

void TimeMeasure::on1msElapsed() {
	for(size_t i = 0; i < TMI_NUMBER; i++) {
		timeMeasure[i].endAudioLoop();
	}
}

TimeMeasure::TimeMeasure()
    : index(0),
      time_sum_between_reset(0),
      time_sum(0),
      time_sum_per_loop(0),
      time_max_between_reset(0),
      time_max(0),
      begin_time(0),
      isMeasuring(false) {
	memset(otherTimeMeasureState, 0, sizeof(otherTimeMeasureState));
	for(size_t i = 0; i < TMI_NUMBER; i++) {
		if(this == &timeMeasure[i]) {
			index = i;
			break;
		}
	}
}

void TimeMeasure::beginMeasure(bool stateSave) {
	__disable_irq();
	uint32_t current_time = TIM2->CNT;

	// Pause others
	for(size_t i = 0; i < TMI_NUMBER; i++) {
		otherTimeMeasureState[i] = timeMeasure[i].updateMeasureAndStop(current_time);
	}

	updateBeginAndStart(current_time);
	current_measure_sum = 0;

	__enable_irq();
}

void TimeMeasure::endMeasure(bool stateRestore) {
	__disable_irq();
	uint32_t current_time = TIM2->CNT;

	if(!updateMeasureAndStop(current_time))
		goto end;

	// Restart paused measures
	for(size_t i = 0; i < TMI_NUMBER; i++) {
		if(otherTimeMeasureState[i]) {
			timeMeasure[i].begin_time = current_time;
			timeMeasure[i].isMeasuring = true;
		}
	}
end:
	time_sum_per_loop += current_measure_sum;
	__enable_irq();
}

bool TimeMeasure::updateBeginAndStart(uint32_t current_time) {
	if(isMeasuring)
		return false;

	begin_time = current_time;
	isMeasuring = true;

	return true;
}

bool TimeMeasure::updateMeasureAndStop(uint32_t current_time) {
	if(!isMeasuring)
		return false;

	uint32_t time_measured = current_time - begin_time;
	time_sum += time_measured;
	current_measure_sum += time_measured;

	isMeasuring = false;

	return true;
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

static uint32_t atomicReadReset(uint32_t* variable) {
	uint32_t value;
	__disable_irq();
	value = *variable;
	*variable = 0;
	__enable_irq();

	return value;
}

uint32_t TimeMeasure::getCumulatedTimeUsAndReset() {
	uint32_t current_time = TIM2->CNT;
	float elapsed_time = current_time - time_sum_between_reset;
	time_sum_between_reset = current_time;
	return atomicReadReset(&time_sum) * 1000000.0f / elapsed_time;
}

uint32_t TimeMeasure::getMaxTimeUsAndReset() {
	uint32_t current_time = TIM2->CNT;
	float elapsed_time = current_time - time_max_between_reset;
	time_max_between_reset = current_time;

	return atomicReadReset(&time_max) * 1000000.0f / elapsed_time;
}

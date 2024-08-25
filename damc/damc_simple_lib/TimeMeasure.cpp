#include "TimeMeasure.h"
#include <main.h>
#include <stm32f7xx.h>
#include <stm32f7xx_hal_gpio.h>
#include <string.h>

TimeMeasure TimeMeasure::timeMeasure[TMI_NUMBER];

static TimeMeasure* stackRunningTasks[16];
static int32_t stackRunningTasksIndex = -1;

static GPIO_TypeDef* const DEBUG_GPIO_PORT[] = {
    [TMI_UsbInterrupt] = STMOD_UART4_RXD_s_GPIO_Port,
    [TMI_AudioProcessing] = STMOD_TIM2_CH1_2_ETR_GPIO_Port,
    [TMI_OtherIRQ] = STMOD_UART4_RXD_GPIO_Port,
    [TMI_MainLoop] = STMOD_UART4_TXD_GPIO_Port,
};

static const uint32_t DEBUG_GPIO_PIN[] = {
    [TMI_UsbInterrupt] = STMOD_UART4_RXD_s_Pin,
    [TMI_AudioProcessing] = STMOD_TIM2_CH1_2_ETR_Pin,
    [TMI_OtherIRQ] = STMOD_UART4_RXD_Pin,
    [TMI_MainLoop] = STMOD_UART4_TXD_Pin,
};

TimeMeasure::TimeMeasure()
    : index(0),
      current_measure_sum(0),
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

void TimeMeasure::beginMeasure() {
	__disable_irq();
	uint32_t current_time = TIM2->CNT;
	HAL_GPIO_WritePin(DEBUG_GPIO_PORT[index], DEBUG_GPIO_PIN[index], GPIO_PIN_SET);

	int32_t stackIndex = stackRunningTasksIndex;

	// Pause running measure
	if(stackIndex >= 0) {
		stackRunningTasks[stackIndex]->updateMeasureAndStop(current_time);
	}

	stackIndex++;
	stackRunningTasksIndex = stackIndex;
	stackRunningTasks[stackIndex] = this;

	begin_time = current_time;
	current_measure_sum = 0;
	__enable_irq();

	isMeasuring = true;
}

void TimeMeasure::endMeasure() {
	isMeasuring = false;

	__disable_irq();
	uint32_t current_time = TIM2->CNT;
	HAL_GPIO_WritePin(DEBUG_GPIO_PORT[index], DEBUG_GPIO_PIN[index], GPIO_PIN_RESET);

	updateMeasureAndStop(current_time);

	int32_t stackIndex = --stackRunningTasksIndex;

	// Restart paused measure
	if(stackIndex >= 0)
		stackRunningTasks[stackIndex]->begin_time = current_time;

	time_sum_per_loop += current_measure_sum;
	__enable_irq();
}

bool TimeMeasure::updateMeasureAndStop(uint32_t current_time) {
	uint32_t time_measured = current_time - begin_time;
	time_sum += time_measured;
	current_measure_sum += time_measured;

	return true;
}

void TimeMeasure::on1msElapsed() {
	for(size_t i = 0; i < TMI_NUMBER; i++) {
		timeMeasure[i].endAudioLoop();
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

static uint32_t atomicReadReset(uint32_t* variable, uint32_t* current_time) {
	uint32_t value;

	__disable_irq();
	value = *variable;
	*variable = 0;
	*current_time = TIM2->CNT;
	__enable_irq();

	return value;
}

uint32_t TimeMeasure::getCumulatedTimeUsAndReset() {
	uint32_t current_time;
	uint32_t measure = atomicReadReset(&time_sum, &current_time);

	float elapsed_time = current_time - time_sum_between_reset;
	time_sum_between_reset = current_time;
	return measure * 1000000.0f / elapsed_time;
}

uint32_t TimeMeasure::getMaxTimeUsAndReset() {
	uint32_t current_time;
	uint32_t measure = atomicReadReset(&time_max, &current_time);

	return measure;
}

uint32_t TimeMeasure::getOnGoingDuration() {
	if(!isMeasuring) {
		return 0;
	}

	return TIM2->CNT - begin_time + current_measure_sum;
}
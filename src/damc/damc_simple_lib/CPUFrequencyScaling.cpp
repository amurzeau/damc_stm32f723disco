#include "CPUFrequencyScaling.h"
#include "AudioCApi.h"
#include "TimeMeasure.h"
#include "stm32n657xx.h"
#include <atomic>

static uint32_t roundToLowerPowerOf2(uint32_t target_value) {
	// Round to lower power of two
	uint32_t divider = 1;
	while(divider * 2 <= target_value)
		divider *= 2;

	return divider;
}

static uint32_t clampDivider(uint32_t divider, uint32_t max, bool power_of_2) {
	if(divider > max) {
		divider = max;
	} else if(divider < 1) {
		divider = 1;
	}
	if(!power_of_2)
		return divider;
	else
		return roundToLowerPowerOf2(divider);
}

static uint32_t getBitPosition(uint32_t value) {
	uint32_t res = 0;

	if(!value) {
		return 32;
	}

	// Left shift 1 and check each bit
	while((value & 1) == 0) {
		value = value >> 1;
		res++;
	}

	return res;
}

#ifdef STM32F723xx
#include <stm32f7xx_hal.h>

uint32_t getHPREValueFromDivider(uint32_t divider) {
	switch(divider) {
		case 1:
			return 0;
		case 2:
			return 0b1000;
		case 4:
			return 0b1001;
		case 8:
			return 0b1010;
	}

	while(1)
		;
}

uint32_t getPPREValueFromDivider(uint32_t divider) {
	switch(divider) {
		case 1:
			return 0;
		case 2:
			return 0b100;
		case 4:
			return 0b101;
		case 8:
			return 0b110;
		case 16:
			return 0b111;
	}

	while(1)
		;
}

void setRawAHBDivider(uint32_t current_ahb_divider, uint32_t ahb_divider) {
	if(ahb_divider < current_ahb_divider) {
		// Increase flash wait states
		uint32_t flash_latency = (216 / ahb_divider - 1) / 30;
		__HAL_FLASH_SET_LATENCY(flash_latency);
		if(__HAL_FLASH_GET_LATENCY() != flash_latency) {
			// Failed to set the latency ?
			while(1)
				;
		}
	}

	// Compute new register RCC->CFGR value to apply new AHB, APB1 and APB2 dividers
	uint32_t apb1_divider = 8 / ahb_divider;
	uint32_t apb2_divider = 16 / ahb_divider;

	uint32_t rcc_cfgr = (getHPREValueFromDivider(ahb_divider) << RCC_CFGR_HPRE_Pos) |
	                    (getPPREValueFromDivider(apb1_divider) << RCC_CFGR_PPRE1_Pos) |
	                    (getPPREValueFromDivider(apb2_divider) << RCC_CFGR_PPRE2_Pos);

	// Update AHB, APB1 and APB2 dividers
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk, rcc_cfgr);

	if((READ_REG(RCC->CFGR) & (RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk)) != rcc_cfgr) {
		// Failed to set prescalers
		while(1)
			;
	}

	if(ahb_divider > current_ahb_divider) {
		// Reduce flash wait states
		uint32_t flash_latency = (216 / ahb_divider - 1) / 30;
		__HAL_FLASH_SET_LATENCY(flash_latency);
		if(__HAL_FLASH_GET_LATENCY() != flash_latency) {
			// Failed to set the latency ?
			while(1)
				;
		}
	}

	/* Update the SystemCoreClock global variable */
	SystemCoreClock = HAL_RCC_GetSysClockFreq() / ahb_divider;
	HAL_InitTick(TICK_INT_PRIORITY);
}

void CPUFrequencyScaling::setAHBDivider(uint32_t divider) {
	// Max divider: 8 for minimal APB1 frequency of 27Mhz
	if(divider > 4) {
		divider = 4;
	} else if(divider < 1) {
		divider = 1;
	}

	// Round to lower power of two
	uint32_t ahb_divider = 1;
	while(ahb_divider * 2 <= divider)
		ahb_divider *= 2;

	// Check if already at that divider
	if(ahb_divider == current_ahb_divider)
		return;

	setRawAHBDivider(current_ahb_divider, ahb_divider);

	current_ahb_divider = ahb_divider;
	recent_ahb_divider_change = true;
	uv_async_send(&asyncFrequencyChanged);

	// Reset cpu usage stats
	cpu_usage_points = 0;
	max_cpu_usage_ratio_per_thousand = 0;
}

#elif defined(STM32N657xx)
#include <stm32n6xx_hal.h>
#include <stm32n6xx_hal_cortex.h>
#include <stm32n6xx_hal_rcc.h>

static void updateTimerPrescaler() {
	uint32_t timer_frequency = HAL_RCC_GetSysClockFreq() / (1UL << LL_RCC_GetTIMPrescaler());

	uint32_t prescaler = (timer_frequency / 1000000) - 1;
	if(prescaler == TIM2->PSC)
		return;

	TIM2->PSC = prescaler;
	TIM2->EGR |= TIM_EGR_UG;
}

void CPUFrequencyScaling::setRawCPUDivider(uint32_t divider) {
	divider = clampDivider(divider, 256, false);

	if(divider == LL_RCC_IC1_GetDivider())
		return;

	// Set sys_cpu_ck frequency
	LL_RCC_IC1_SetDivider(divider);

	/* Update the SystemCoreClock global variable */
	SystemCoreClock = HAL_RCC_GetCpuClockFreq();
	HAL_SYSTICK_Config(SystemCoreClock / (1000UL / (uint32_t) uwTickFreq));

	uv_async_send(&asyncFrequencyChanged);
}

void CPUFrequencyScaling::setRawAXIDivider(uint32_t divider) {
	divider = clampDivider(divider, 256, false);

	if(divider == LL_RCC_IC2_GetDivider())
		return;

	// Set AXI frequency divider
	LL_RCC_IC2_SetDivider(divider);

	updateTimerPrescaler();

	uv_async_send(&asyncFrequencyChanged);
}

static uint32_t getAHBDivider() {
	return 1 << (LL_RCC_GetAHBPrescaler() >> RCC_CFGR2_HPRE_Pos);
}

void CPUFrequencyScaling::setRawAHBDivider(uint32_t divider) {
	divider = clampDivider(divider, 16, true);

	if(divider == getAHBDivider())
		return;

	// Set AHB frequency divider
	LL_RCC_SetAHBPrescaler(getBitPosition(divider) << RCC_CFGR2_HPRE_Pos);

	uv_async_send(&asyncFrequencyChanged);
}

static uint32_t getAPBDivider(uint32_t index) {
	uint32_t register_bit_offset = 4 * (index - 1);
	return 1 << ((RCC->CFGR2 >> register_bit_offset) & 7);
}

void CPUFrequencyScaling::setRawAPBDivider(uint32_t index, uint32_t divider) {
	divider = clampDivider(divider, 128, true);

	uint32_t register_bit_offset = 4 * (index - 1);

	if(divider == getAPBDivider(index))
		return;

	// Set APB frequency divider
	MODIFY_REG(RCC->CFGR2, 7 << register_bit_offset, getBitPosition(divider) << register_bit_offset);

	uv_async_send(&asyncFrequencyChanged);
}

static uint32_t getTimerDivider() {
	return 1 << LL_RCC_GetTIMPrescaler();
}

void CPUFrequencyScaling::setRawTimerDivider(uint32_t index, uint32_t divider) {
	divider = clampDivider(divider, 8, true);

	if(index != 1)
		return;

	if(divider == getTimerDivider())
		return;

	// Set Timer frequency divider
	LL_RCC_SetTIMPrescaler(getBitPosition(divider));

	updateTimerPrescaler();

	uv_async_send(&asyncFrequencyChanged);
}

#endif

CPUFrequencyScaling::CPUFrequencyScaling(OscRoot* oscRoot)
    : OscContainer(oscRoot, "cpu"),
      oscRoot(oscRoot),
      oscManualControl(this, "manual", true),
      oscPllFrequency(this, "freq", HAL_RCCEx_GetPLL1CLKFreq()),
      oscCpuFrequency(this, "cpuFreq", SystemCoreClock),
      oscAXIFrequency(this, "axiFreq", HAL_RCC_GetSysClockFreq()),
      oscAHBFrequency(this, "ahbFreq", HAL_RCC_GetHCLKFreq()),
      oscAPB1Frequency(this, "apb1Freq", HAL_RCC_GetPCLK1Freq()),
      oscAPB2Frequency(this, "apb2Freq", HAL_RCC_GetPCLK2Freq()),
      oscAPB4Frequency(this, "apb4Freq", HAL_RCC_GetPCLK4Freq()),
      oscAPB5Frequency(this, "apb5Freq", HAL_RCC_GetPCLK5Freq()),
      oscTimerFrequency(this, "timerFreq", HAL_RCC_GetSysClockFreq() / getTimerDivider()),
      // Don't persist this variable as we use it also as a readonly variable when "manual" == false
      oscCpuDivider(this, "cpuDivider", LL_RCC_IC1_GetDivider(), false),
      oscAXIDivider(this, "axiDivider", LL_RCC_IC2_GetDivider(), false),
      oscAHBDivider(this, "ahbDivider", getAHBDivider(), false),
      oscAPB1Divider(this, "apb1Divider", getAPBDivider(1), false),
      oscAPB2Divider(this, "apb2Divider", getAPBDivider(2), false),
      oscAPB4Divider(this, "apb4Divider", getAPBDivider(4), false),
      oscAPB5Divider(this, "apb5Divider", getAPBDivider(5), false),
      oscTimerDivider(this, "timerDivider", getTimerDivider(), false),
      current_ahb_divider(1),
      recent_ahb_divider_change(false),
      max_cpu_usage_ratio_per_thousand(0),
      cpu_usage_points(0),
      cpu_usage_points_target(100) {
	uv_async_init(uv_default_loop(), &asyncFrequencyChanged, CPUFrequencyScaling::onFrequencyChanged);
	asyncFrequencyChanged.data = this;

#define DEFINE_CHANGE_CALLBACK(dividerChangeFunction, ...) \
	[this](int32_t value) { \
		if(oscManualControl) { \
			dividerChangeFunction; \
		} else { \
			oscCpuDivider.set(current_ahb_divider); \
		} \
	}

	oscCpuDivider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawCPUDivider(value)));
	oscAXIDivider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawAXIDivider(value)));
	oscAHBDivider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawAHBDivider(value)));
	oscAPB1Divider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawAPBDivider(1, value)));
	oscAPB2Divider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawAPBDivider(2, value)));
	oscAPB4Divider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawAPBDivider(4, value)));
	oscAPB5Divider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawAPBDivider(5, value)));
	oscTimerDivider.addChangeCallback(DEFINE_CHANGE_CALLBACK(setRawTimerDivider(1, value)));

	resetFrequencyToMaxPerformance();
}

CPUFrequencyScaling::~CPUFrequencyScaling() {}

void CPUFrequencyScaling::init() {
	oscRoot->addValueChangedCallback([this]() { resetFrequencyToMaxPerformance(); });
}

void CPUFrequencyScaling::resetFrequencyToMaxPerformance() {
	// If manual mode, don't auto adjust frequency
	if(oscManualControl)
		return;

	max_cpu_usage_ratio_per_thousand = 0;
	cpu_usage_points = 0;
	cpu_usage_points_target = 100;

	// Go back to max performance in case an option were enabled needing the extra performance
	adjustCpuFreq(CpuFreqAdjustement::ResetToMax);
}

void CPUFrequencyScaling::updateCpuUsage() {
	// If manual mode, don't auto adjust frequency
	if(oscManualControl)
		return;

	// Skip first measure after a frequency change
	if(recent_ahb_divider_change) {
		recent_ahb_divider_change = false;
		return;
	}

	// Ensure previous code calling updateCpuUsage() is not reordered after the measure
	std::atomic_signal_fence(std::memory_order_seq_cst);

	// CPU time in realtime interrupts
	uint32_t cpu_usage_ratio_us = TimeMeasure::timeMeasure[TMI_UsbInterrupt].getMaxTimeUs() +
	                              TimeMeasure::timeMeasure[TMI_AudioProcessing].getMaxTimeUs();

	// Duration of the main loop task so far (if its too long, we will also increase CPU frequency to speed it up)
	uint32_t main_loop_task_duration_us = TimeMeasure::timeMeasure[TMI_MainLoop].getOnGoingDuration();

	cpu_usage_points++;
	if(cpu_usage_ratio_us > max_cpu_usage_ratio_per_thousand) {
		max_cpu_usage_ratio_per_thousand = cpu_usage_ratio_us;

		// We got a new maximum, wait for longer until we have no more maximum
		cpu_usage_points = 0;
	}

	uint32_t current_needed_cpu_usage_room = 25 * current_ahb_divider;

	if(main_loop_task_duration_us > 10000) {
		// If main loop current task is running so far for more than 10ms, increase CPU frequency to max
		// to speed it up.
		adjustCpuFreq(CpuFreqAdjustement::ResetToMax);
	} else if(max_cpu_usage_ratio_per_thousand > 900 - current_needed_cpu_usage_room) {
		// We are above 90% + 2.5% normalized room cpu usage, increase cpu speed
		adjustCpuFreq(CpuFreqAdjustement::IncreaseSpeed);
	} else if(cpu_usage_points >= cpu_usage_points_target &&
	          (2 * max_cpu_usage_ratio_per_thousand < 850 - current_needed_cpu_usage_room * 2)) {
		// We can divide cpu frequency by 2 while being under 85% cpu usage + 2.5% normalized room
		adjustCpuFreq(CpuFreqAdjustement::DecreaseSpeed);
	}

	if(cpu_usage_points >= cpu_usage_points_target) {
		// Reset stats even if we didn't changed the divider to update the needed frequency if less speed is needed
		// later
		cpu_usage_points = 0;
		max_cpu_usage_ratio_per_thousand = 0;
	}
}

void CPUFrequencyScaling::adjustCpuFreq(CpuFreqAdjustement adjustment) {}

void CPUFrequencyScaling::onFrequencyChanged(uv_async_t* handle) {
	CPUFrequencyScaling* thisInstance = (CPUFrequencyScaling*) handle->data;

	thisInstance->oscCpuDivider.set(LL_RCC_IC1_GetDivider());
	thisInstance->oscAXIDivider.set(LL_RCC_IC2_GetDivider());
	thisInstance->oscAHBDivider.set(getAHBDivider());
	thisInstance->oscAPB1Divider.set(getAPBDivider(1));
	thisInstance->oscAPB2Divider.set(getAPBDivider(2));
	thisInstance->oscAPB4Divider.set(getAPBDivider(4));
	thisInstance->oscAPB5Divider.set(getAPBDivider(5));
	thisInstance->oscTimerDivider.set(getTimerDivider());

	thisInstance->oscCpuFrequency.set(SystemCoreClock);
	thisInstance->oscAXIFrequency.set(HAL_RCC_GetSysClockFreq());
	thisInstance->oscAHBFrequency.set(HAL_RCC_GetHCLKFreq());
	thisInstance->oscAPB1Frequency.set(HAL_RCC_GetPCLK1Freq());
	thisInstance->oscAPB2Frequency.set(HAL_RCC_GetPCLK2Freq());
	thisInstance->oscAPB4Frequency.set(HAL_RCC_GetPCLK4Freq());
	thisInstance->oscAPB5Frequency.set(HAL_RCC_GetPCLK5Freq());
	thisInstance->oscTimerFrequency.set(HAL_RCC_GetSysClockFreq() / getTimerDivider());
}
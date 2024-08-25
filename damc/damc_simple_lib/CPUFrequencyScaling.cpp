#include "CPUFrequencyScaling.h"
#include "AudioCApi.h"
#include "TimeMeasure.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_rcc.h>

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

CPUFrequencyScaling::CPUFrequencyScaling(OscRoot* oscRoot)
    : OscContainer(oscRoot, "cpu"),
      oscRoot(oscRoot),
      oscManualControl(this, "manual", false),
      oscCurrentFrequency(this, "freq", SystemCoreClock),
      // Don't persist this variable as we use it also as a readonly variable when "manual" == false
      oscCpuDivider(this, "divider", 1, false),
      current_ahb_divider(1),
      recent_ahb_divider_change(false),
      max_cpu_usage_ratio_per_thousand(0),
      cpu_usage_points(0),
      cpu_usage_points_target(100) {
	uv_async_init(uv_default_loop(), &asyncFrequencyChanged, CPUFrequencyScaling::onFrequencyChanged);
	asyncFrequencyChanged.data = this;

	oscCpuDivider.addChangeCallback([this](int32_t value) {
		if(oscManualControl) {
			setAHBDivider(value);
		} else {
			oscCpuDivider.set(current_ahb_divider);
		}
	});

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
	setAHBDivider(1);
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

	// Ensure previous code is not reordered after the measure
	__DSB();

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
		setAHBDivider(1);
	} else if(max_cpu_usage_ratio_per_thousand > 900 - current_needed_cpu_usage_room) {
		// We are above 90% + 2.5% normalized room cpu usage, increase cpu speed
		setAHBDivider(current_ahb_divider / 2);
	} else if(cpu_usage_points >= cpu_usage_points_target &&
	          (2 * max_cpu_usage_ratio_per_thousand < 850 - current_needed_cpu_usage_room * 2)) {
		// We can divide cpu frequency by 2 while being under 85% cpu usage + 2.5% normalized room
		setAHBDivider(current_ahb_divider * 2);
	}

	if(cpu_usage_points >= cpu_usage_points_target) {
		// Reset stats even if we didn't changed the divider to update the needed frequency if less speed is needed
		// later
		cpu_usage_points = 0;
		max_cpu_usage_ratio_per_thousand = 0;
	}
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

	current_ahb_divider = ahb_divider;
	recent_ahb_divider_change = true;
	uv_async_send(&asyncFrequencyChanged);

	// Reset cpu usage stats
	cpu_usage_points = 0;
	max_cpu_usage_ratio_per_thousand = 0;
}

void CPUFrequencyScaling::onFrequencyChanged(uv_async_t* handle) {
	CPUFrequencyScaling* thisInstance = (CPUFrequencyScaling*) handle->data;

	thisInstance->oscCpuDivider.set(thisInstance->current_ahb_divider);
	thisInstance->oscCurrentFrequency.set(SystemCoreClock);
}
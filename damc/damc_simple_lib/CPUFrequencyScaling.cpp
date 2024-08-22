#include "CPUFrequencyScaling.h"
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
      oscCurrentFrequency(this, "freq"),
      current_ahb_divider(1),
      max_cpu_usage_ratio_per_million(0),
      cpu_usage_points(0),
      cpu_usage_points_target(0) {
	resetFrequencyToMaxPerformance();
}

CPUFrequencyScaling::~CPUFrequencyScaling() {}

void CPUFrequencyScaling::init() {
	oscRoot->addValueChangedCallback([this]() { resetFrequencyToMaxPerformance(); });
}

void CPUFrequencyScaling::resetFrequencyToMaxPerformance() {
	max_cpu_usage_ratio_per_million = 0;
	cpu_usage_points = 0;
	cpu_usage_points_target = 4;

	// Go back to max performance in case an option were enabled needing the extra performance
	setAHBDivider(1);
}

void CPUFrequencyScaling::notifyCurrentCpuUsage(uint32_t cpu_usage_ratio_per_million) {
	cpu_usage_points++;
	if(cpu_usage_ratio_per_million > max_cpu_usage_ratio_per_million) {
		max_cpu_usage_ratio_per_million = cpu_usage_ratio_per_million;

		// We got a new maximum, wait for longer until we have no more maximum
		cpu_usage_points = 0;
	}

	if(max_cpu_usage_ratio_per_million > 850000) {
		// We are above 85% cpu usage, increase cpu speed
		setAHBDivider(current_ahb_divider / 2);
	}

	if(cpu_usage_points >= cpu_usage_points_target) {
		// Adjust frequency scaling to target max 80% cpu usage
		uint32_t divider = current_ahb_divider * 800000 / max_cpu_usage_ratio_per_million;
		setAHBDivider(divider);

		// Reset stats even if we didn't changed the divider
		cpu_usage_points = 0;
		max_cpu_usage_ratio_per_million = 0;
	}
}

uint32_t CPUFrequencyScaling::setAHBDivider(uint32_t divider) {
	// Max divider: 8 for minimal APB1 frequency of 27Mhz
	if(divider > 8) {
		divider = 8;
	} else if(divider < 1) {
		divider = 1;
	}

	// Round to lower power of two
	uint32_t ahb_divider = 1;
	while(ahb_divider * 2 <= divider)
		ahb_divider *= 2;

	// Check if already at that divider
	if(ahb_divider == current_ahb_divider)
		return ahb_divider;

	// Compute new register RCC->CFGR value to apply new AHB, APB1 and APB2 dividers
	uint32_t apb1_divider = 8 / ahb_divider;
	uint32_t apb2_divider = 16 / ahb_divider;

	uint32_t rcc_cfgr = (getHPREValueFromDivider(ahb_divider) << RCC_CFGR_HPRE_Pos) |
	                    (getPPREValueFromDivider(apb1_divider) << RCC_CFGR_PPRE1_Pos) |
	                    (getPPREValueFromDivider(apb2_divider) << RCC_CFGR_PPRE2_Pos);

	// Update AHB, APB1 and APB2 dividers
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk, rcc_cfgr);

	/* Update the SystemCoreClock global variable */
	SystemCoreClock = HAL_RCC_GetSysClockFreq() / ahb_divider;
	HAL_InitTick(TICK_INT_PRIORITY);

	current_ahb_divider = ahb_divider;

	// Reset cpu usage stats
	cpu_usage_points = 0;
	max_cpu_usage_ratio_per_million = 0;

	oscCurrentFrequency.set(SystemCoreClock);

	return ahb_divider;
}

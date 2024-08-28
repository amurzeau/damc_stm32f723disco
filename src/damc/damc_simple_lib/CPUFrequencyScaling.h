#pragma once

#include <Osc/OscReadOnlyVariable.h>
#include <Osc/OscVariable.h>
#include <OscRoot.h>
#include <stdint.h>
#include <uv.h>

class CPUFrequencyScaling : public OscContainer {
public:
	CPUFrequencyScaling(OscRoot* oscRoot);
	~CPUFrequencyScaling();

	void init();
	void resetFrequencyToMaxPerformance();
	void updateCpuUsage();

protected:
	void setAHBDivider(uint32_t divider);
	static void onFrequencyChanged(uv_async_t* handle);

private:
	OscRoot* oscRoot;
	OscVariable<bool> oscManualControl;
	OscReadOnlyVariable<int32_t> oscCurrentFrequency;
	OscVariable<int32_t> oscCpuDivider;

	uint32_t current_ahb_divider;
	bool recent_ahb_divider_change;

	uint32_t max_cpu_usage_ratio_per_thousand;
	uint32_t cpu_usage_points;
	uint32_t cpu_usage_points_target;

	uv_async_t asyncFrequencyChanged;
};

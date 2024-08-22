#pragma once

#include <Osc/OscReadOnlyVariable.h>
#include <OscRoot.h>
#include <stdint.h>
#include <uv.h>

class CPUFrequencyScaling : public OscContainer {
public:
	CPUFrequencyScaling(OscRoot* oscRoot);
	~CPUFrequencyScaling();

	void init();
	void resetFrequencyToMaxPerformance();
	void notifyCurrentCpuUsage(uint32_t cpu_usage_ratio_per_million);

protected:
	uint32_t setAHBDivider(uint32_t divider);

private:
	OscRoot* oscRoot;
	OscReadOnlyVariable<int32_t> oscCurrentFrequency;
	uint32_t current_ahb_divider;
	bool recent_ahb_divider_change;

	uint32_t max_cpu_usage_ratio_per_million;
	uint32_t cpu_usage_points;
	uint32_t cpu_usage_points_target;
};

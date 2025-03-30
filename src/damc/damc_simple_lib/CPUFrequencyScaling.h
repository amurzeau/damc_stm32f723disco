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
	enum class CpuFreqAdjustement {
		ResetToMax,
		IncreaseSpeed,
		DecreaseSpeed,
	};
	void adjustCpuFreq(CpuFreqAdjustement adjustment);
	static void onFrequencyChanged(uv_async_t* handle);

	void setRawCPUDivider(uint32_t divider);
	void setRawAXIDivider(uint32_t divider);
	void setRawAHBDivider(uint32_t divider);
	void setRawAPBDivider(uint32_t index, uint32_t divider);
	void setRawTimerDivider(uint32_t index, uint32_t divider);

private:
	OscRoot* oscRoot;
	OscVariable<bool> oscManualControl;

	OscReadOnlyVariable<int32_t> oscPllFrequency;
	OscReadOnlyVariable<int32_t> oscCpuFrequency;
	OscReadOnlyVariable<int32_t> oscAXIFrequency;
	OscReadOnlyVariable<int32_t> oscAHBFrequency;
	OscReadOnlyVariable<int32_t> oscAPB1Frequency;
	OscReadOnlyVariable<int32_t> oscAPB2Frequency;
	OscReadOnlyVariable<int32_t> oscAPB4Frequency;
	OscReadOnlyVariable<int32_t> oscAPB5Frequency;
	OscReadOnlyVariable<int32_t> oscTimerFrequency;

	OscVariable<int32_t> oscCpuDivider;
	OscVariable<int32_t> oscAXIDivider;
	OscVariable<int32_t> oscAHBDivider;
	OscVariable<int32_t> oscAPB1Divider;
	OscVariable<int32_t> oscAPB2Divider;
	OscVariable<int32_t> oscAPB4Divider;
	OscVariable<int32_t> oscAPB5Divider;
	OscVariable<int32_t> oscTimerDivider;

	uint32_t current_ahb_divider;
	bool recent_ahb_divider_change;

	uint32_t max_cpu_usage_ratio_per_thousand;
	uint32_t cpu_usage_points;
	uint32_t cpu_usage_points_target;

	uv_async_t asyncFrequencyChanged;
};

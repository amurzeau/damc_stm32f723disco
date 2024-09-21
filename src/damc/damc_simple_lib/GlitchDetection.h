#pragma once

#ifdef __cplusplus
extern "C" {
#endif

enum GlitchType {
	GT_UsbIsochronousTransferLost,
	GT_UsbOutOverrun,
	GT_UsbOutUnderrun,
	GT_UsbInOverrun,
	GT_AudioProcessInterruptLost,
	GT_CodecOutXRun,
	GT_CodecOutDmaUnderrun,
	GT_CodecInXRun,
	GT_CodecInDmaOverrun,

	GT_Number
};

void GLITCH_DETECTION_increment_counter(enum GlitchType type) __attribute__((optimize("-O1")));

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "Osc/OscReadOnlyVariable.h"
#include "Osc/OscVariable.h"
#include <Osc/OscContainer.h>
#include <array>
#include <uv.h>

class GlitchDetection : OscContainer {
public:
	GlitchDetection(OscContainer* parent);

	void start();

protected:
	static void onUpdateTimer(uv_timer_t* handle);

private:
	std::array<OscReadOnlyVariable<int32_t>, GT_Number> oscGlitchCounters;
	OscVariable<bool> oscResetCounters;
	uv_timer_t updateTimer;
	size_t nextIndexToUpdate;
};

#endif
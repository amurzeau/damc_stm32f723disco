#include "GlitchDetection.h"
#include "usbd_conf.h"
#include "uv.h"
#include <atomic>
#include <main.h>
#include <string.h>

static std::atomic_int32_t glitches_counters[GT_Number];
static std::atomic_bool feedback_read_by_host[AUDIO_OUT_NUMBER];

void GLITCH_DETECTION_increment_counter(enum GlitchType type) {
	// HAL_GPIO_WritePin(STMOD_UART4_TXD_GPIO_Port, STMOD_UART4_TXD_Pin, GPIO_PIN_SET);

	// Relaxed atomic: no dependency on other variables.
	glitches_counters[type].fetch_add(1, std::memory_order_relaxed);

	// HAL_GPIO_WritePin(STMOD_UART4_TXD_GPIO_Port, STMOD_UART4_TXD_Pin, GPIO_PIN_RESET);
}

void GLITCH_DETECTION_set_USB_out_feedback_state(int outIndex, bool feedback_working) {
	// Relaxed atomic: no dependency on other variables.
	feedback_read_by_host[outIndex].store(feedback_working, std::memory_order_relaxed);
}

GlitchDetection::GlitchDetection(OscContainer* parent)
    : OscContainer(parent, "glitches"),
      oscGlitchCounters{
          OscReadOnlyVariable<int32_t>{this, "usbIsochronousTransferLost"},
          OscReadOnlyVariable<int32_t>{this, "usbOutOverrun"},
          OscReadOnlyVariable<int32_t>{this, "usbOutUnderrun"},
          OscReadOnlyVariable<int32_t>{this, "usbInOverrun"},
          OscReadOnlyVariable<int32_t>{this, "audioProcessInterruptLost"},
          OscReadOnlyVariable<int32_t>{this, "codecOutXRun"},
          OscReadOnlyVariable<int32_t>{this, "codecOutDmaUnderrun"},
          OscReadOnlyVariable<int32_t>{this, "codecInXRun"},
          OscReadOnlyVariable<int32_t>{this, "codecInDmaOverrun"},
      },
      oscFeedbackReadByHost{
          OscReadOnlyVariable<bool>(this, "feedbackSyncOut0", false),
          OscReadOnlyVariable<bool>(this, "feedbackSyncOut1", false),
      },
      oscResetCounters(this, "reset", false, false),
      nextIndexToUpdate(0) {
	uv_timer_init(uv_default_loop(), &updateTimer);
	updateTimer.data = this;
}

void GlitchDetection::start() {
	uv_timer_start(&updateTimer, &onUpdateTimer, 200, 200);

	oscResetCounters.addChangeCallback([](bool value) {
		if(value) {
			// Concurrent reset with increments from ISRs
			// Use 32 bits atomic writes instead of memcpy.
			for(std::atomic_int32_t& glitches_counter : glitches_counters) {
				glitches_counter.store(0, std::memory_order_relaxed);
			}
		}
	});
}

void GlitchDetection::onUpdateTimer(uv_timer_t* handle) {
	GlitchDetection* thisInstance = (GlitchDetection*) handle->data;

	size_t nextIndexToUpdate = thisInstance->nextIndexToUpdate;

	thisInstance->oscGlitchCounters[nextIndexToUpdate].set(glitches_counters[nextIndexToUpdate]);
	thisInstance->nextIndexToUpdate = (nextIndexToUpdate + 1) % GT_Number;

	for(size_t i = 0; i < AUDIO_OUT_NUMBER; i++) {
		// Relaxed atomic: no dependency on other variables.
		thisInstance->oscFeedbackReadByHost[i] = feedback_read_by_host[i].load(std::memory_order_relaxed);
	}
}
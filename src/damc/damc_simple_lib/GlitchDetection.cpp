#include "GlitchDetection.h"
#include "uv.h"
#include <main.h>
#include <stm32f7xx.h>
#include <stm32f7xx_hal_gpio.h>
#include <string.h>

static int32_t glitches_counters[GT_Number];

void GLITCH_DETECTION_increment_counter(enum GlitchType type) {
	// HAL_GPIO_WritePin(STMOD_UART4_TXD_GPIO_Port, STMOD_UART4_TXD_Pin, GPIO_PIN_SET);
	glitches_counters[type]++;
	// HAL_GPIO_WritePin(STMOD_UART4_TXD_GPIO_Port, STMOD_UART4_TXD_Pin, GPIO_PIN_RESET);
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
      oscResetCounters(this, "reset", false, false),
      nextIndexToUpdate(0) {
	uv_timer_init(uv_default_loop(), &updateTimer);
	updateTimer.data = this;
}

void GlitchDetection::start() {
	uv_timer_start(&updateTimer, &onUpdateTimer, 200, 200);

	oscResetCounters.addChangeCallback([](bool value) {
		if(value) {
			memset(glitches_counters, 0, sizeof(glitches_counters));
		}
	});
}

void GlitchDetection::onUpdateTimer(uv_timer_t* handle) {
	GlitchDetection* thisInstance = (GlitchDetection*) handle->data;

	size_t nextIndexToUpdate = thisInstance->nextIndexToUpdate;

	thisInstance->oscGlitchCounters[nextIndexToUpdate].set(glitches_counters[nextIndexToUpdate]);
	thisInstance->nextIndexToUpdate = (nextIndexToUpdate + 1) % GT_Number;
}
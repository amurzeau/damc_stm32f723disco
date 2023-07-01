#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DAMC_init();
void DAMC_processAudioInterleaved(int16_t index, const int16_t* data_input, int16_t* data_output, size_t nframes);
void DAMC_mainLoop();
void DAMC_usbInterruptBeginMeasure();
void DAMC_usbInterruptEndMeasure();

#ifdef __cplusplus
}
#endif

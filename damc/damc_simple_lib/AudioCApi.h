#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DAMC_init();
void DAMC_start();
void DAMC_processAudioInterleaved(const int16_t** input_endpoints, size_t input_endpoints_number, int16_t** output_endpoints, size_t output_endpoints_number, size_t nframes);
void DAMC_mainLoop();
void DAMC_usbInterruptBeginMeasure();
void DAMC_usbInterruptEndMeasure();

#ifdef __cplusplus
}
#endif

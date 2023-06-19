#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void DAMC_init();
void DAMC_processAudioInterleaved(const int16_t* data_input, int16_t* data_output, size_t nframes);
void DAMC_mainLoop();

#ifdef __cplusplus
}
#endif

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DAMC_init();
void DAMC_start();
void DAMC_processAudioInterleaved(const int16_t** input_endpoints,
                                  size_t input_endpoints_number,
                                  int16_t** output_endpoints,
                                  size_t output_endpoints_number,
                                  size_t nframes);
void DAMC_mainLoop();
void DAMC_usbInterruptBeginMeasure();
void DAMC_usbInterruptEndMeasure();
void DAMC_setControlFromUSB(uint8_t unit_id, uint8_t control_type, uint8_t channel, uint8_t bRequest, uint16_t value);
uint16_t DAMC_getControlFromUSB(uint8_t unit_id, uint8_t control_type, uint8_t channel, uint8_t bRequest);

#ifdef __cplusplus
}
#endif

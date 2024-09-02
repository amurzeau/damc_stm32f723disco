#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum TimeMeasureItem {
	TMI_UsbInterrupt,
	TMI_AudioProcessing,
	TMI_OtherIRQ,
	TMI_MainLoop,

	TMI_NUMBER,
};

void DAMC_init();
void DAMC_start();
void DAMC_processAudioInterleaved(const int16_t** input_endpoints,
                                  size_t input_endpoints_number,
                                  int16_t** output_endpoints,
                                  size_t output_endpoints_number,
                                  size_t nframes);
void DAMC_mainLoop();
void DAMC_beginMeasure(enum TimeMeasureItem item);
void DAMC_endMeasure(enum TimeMeasureItem item);
void DAMC_resetFrequencyToMaxPerformance();
void DAMC_setControlFromUSB(
    uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest, uint16_t value);
uint16_t DAMC_getControlFromUSB(uint8_t unit_id, uint8_t control_selector, uint8_t channel, uint8_t bRequest);

enum DAMC_USB_Buffer_e {
	DUB_Out1,
	DUB_Out2,
	DUB_In,
};
uint32_t DAMC_getUSBFeedbackValue(enum DAMC_USB_Buffer_e index);
uint32_t DAMC_getUSBInSizeValue(enum DAMC_USB_Buffer_e index);

void DAMC_writeAudioSample(enum DAMC_USB_Buffer_e index, const void* data, size_t size);
size_t DAMC_readAudioSample(enum DAMC_USB_Buffer_e index, void* data, size_t size);

#ifdef __cplusplus
}
#endif

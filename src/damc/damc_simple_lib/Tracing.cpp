#include "Tracing.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_pcd.h>

#ifdef ENABLE_TRACING

struct history_data {
	uint32_t cycles;
	uint32_t diff_cycles;
	const char* operation;
	uint8_t epnum;
	uint8_t is_in;
	uint32_t DOEPCTL;
	uint32_t DOEPINT;
};

struct TRACING_Instance {
	struct history_data history[1024];
	uint32_t history_index;
};

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
struct TRACING_Instance tracing;

uint32_t offset_count;
void TRACING_add(bool in_ep, uint8_t epnum, const char* operation) {
	struct TRACING_Instance* data;

	data = &tracing;

	data->history_index = (data->history_index + 1) % (sizeof(data->history) / sizeof(data->history[0]));
	struct history_data* history = &data->history[data->history_index];
	history->cycles = TIM2->CNT;
	history->diff_cycles = history->cycles - offset_count;
	history->operation = operation;
	history->is_in = in_ep;
	history->epnum = epnum;

	USB_OTG_GlobalTypeDef* USBx = hpcd_USB_OTG_HS.Instance;
	uint32_t USBx_BASE = (uint32_t) USBx;
	history->DOEPCTL = USBx_OUTEP(epnum)->DOEPCTL;
	history->DOEPINT = USBx_OUTEP(epnum)->DOEPINT;
}

#endif
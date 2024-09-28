#include "Tracing.h"
#include "cmsis_gcc.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_pcd.h>

#ifdef ENABLE_TRACING

struct history_data {
	uint32_t cycles;
	uint32_t diff_cycles;
	const char* operation;
	uint8_t epnum;
	uint8_t is_in;
};

struct TRACING_Instance {
	struct history_data history[1024];
	uint32_t history_index;
};

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
struct TRACING_Instance tracing;

static volatile uint32_t* SCB_DEMCR = (volatile uint32_t*) 0xE000EDFC;  // address of the register

void TRACING_init() {
	uint32_t SWOPrescaler = (216000000 / 27000000) -
	                        1; /* SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock */

	DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;            // Enable trace output
	CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; /* enable trace in core debug */
	*((volatile unsigned*) (ITM_BASE + 0x400F0)) =
	    0x00000002; /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1:
	                   SWO Manchester encoding) */
	*((volatile unsigned*) (ITM_BASE + 0x40010)) =
	    SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
	*((volatile unsigned*) (ITM_BASE + 0x00FB0)) = 0xC5ACCE55; /* ITM Lock Access Register, C5ACCE55 enables more write
	                                                              access to Control Register 0xE00 :: 0xFFC */
	ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk |
	           ITM_TCR_ITMENA_Msk;   /* ITM Trace Control Register */
	ITM->TPR = ITM_TPR_PRIVMASK_Msk; /* ITM Trace Privilege Register */
	ITM->TER = 1; /* ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port. */
	*((volatile unsigned*) (ITM_BASE + 0x01000)) = 0x400003FE; /* DWT_CTRL */
	*((volatile unsigned*) (ITM_BASE + 0x40304)) = 0x00000100; /* Formatter and Flush Control Register */
}

__STATIC_INLINE uint32_t ITM_SendChar32(uint32_t ch) {
	if(((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
	   ((ITM->TER & 1UL) != 0UL))                  /* ITM Port #0 enabled */
	{
		while(ITM->PORT[0U].u32 == 0UL) {
			__NOP();
		}
		ITM->PORT[0U].u32 = ch;
	}
	return (ch);
}

char intToChar(uint8_t value) {
	if(value < 10)
		return value + '0';
	else
		return (value - 10) + 'A';
}

uint32_t offset_count;
void TRACING_add(bool in_ep, uint32_t epnum, const char* operation) {
	struct TRACING_Instance* data;

	data = &tracing;

	data->history_index = (data->history_index + 1) % (sizeof(data->history) / sizeof(data->history[0]));
	struct history_data* history = &data->history[data->history_index];
	history->cycles = TIM2->CNT;
	history->diff_cycles = history->cycles - offset_count;
	history->operation = operation;
	history->is_in = in_ep;
	history->epnum = epnum;

	ITM_SendChar32(epnum);

	const char* p = operation;
	// clang-format off
	while(*p) {
		uint32_t data = 0;
		for(size_t i = 0; i < 4; i++) {
			uint8_t c = *p;
			if(c)
				p++;
			data >>= 8;
			data |= c << 24;
		}

		ITM_SendChar32(data);
	}
	// clang-format on
}

#endif
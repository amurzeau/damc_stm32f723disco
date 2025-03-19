#include "Tracing.h"
#include <string.h>

#ifdef ENABLE_TRACING

#include "cmsis_gcc.h"
#include <stm32f7xx.h>
#include <stm32f7xx_hal_pcd.h>

/**
 * In VSCode, retrieve data using Debug Console tab:
 *  - Run "set print elements 0"
 *  - Clear the console
 *  - Run "p tracing"
 *  - Right click -> Copy all
 *  - Paste in a new VSCode tab
 *  - Use language "JSON" and format it (even if this is stricly JSON)
 */

// #define TRACE_TO_ITM

extern uint32_t original_program_pointer;

struct history_data {
	uint32_t cycles;
	uint32_t diff_cycles;
	const char* operation;
	uint32_t preempted_program_pointer;
	uint8_t epnum;
	uint8_t is_in;

	int32_t diff_usb;
	int32_t expected_buff;
	int32_t adj_usb_buff;
	int32_t usb_buff;
	int32_t dma_pos_at_usb;
	int32_t buffer_processed_flag;
	int32_t dma_isr_flag;
	int32_t available_data_for_processing;
	int32_t usb_available_lock;
};

struct TRACING_Instance {
	struct history_data history[1024];
	uint32_t history_index;
};

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
struct TRACING_Instance tracing;
uint32_t offset_count;

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

void TRACING_reset() {
	tracing.history_index = 0;
	offset_count = TIM2->CNT;
}

void TRACING_gpio_set(bool value) {
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

static uint32_t TRACING_getNewIndex() {
	struct TRACING_Instance* data;

	data = &tracing;

	uint32_t expected_old_index;
	bool success;
	do {
		expected_old_index = data->history_index;
		uint32_t new_index = (expected_old_index + 1) % (sizeof(data->history) / sizeof(data->history[0]));
		success = __atomic_compare_exchange_n(
		    &data->history_index, &expected_old_index, new_index, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED);
	} while(!success);

	return expected_old_index;
}

#ifndef TRACING_add
void TRACING_add(bool in_ep, uint32_t epnum, const char* operation) {
	struct TRACING_Instance* data;

	data = &tracing;

	uint32_t history_index = TRACING_getNewIndex();
	struct history_data* history = &data->history[history_index];
	memset(history, 0, sizeof(*history));

	history->cycles = TIM2->CNT;
	history->diff_cycles = history->cycles - offset_count;
	history->operation = operation;
	history->preempted_program_pointer = original_program_pointer;
	history->is_in = in_ep;
	history->epnum = epnum;

#ifdef TRACE_TO_ITM
	ITM_SendChar32(epnum | (in_ep << 7));

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
#endif
}
#endif

void TRACING_add_buffer(uint8_t index, uint32_t data_available, int32_t dma_pos, const char* operation) {
	struct TRACING_Instance* data;

	data = &tracing;

	uint32_t history_index = TRACING_getNewIndex();
	struct history_data* history = &data->history[history_index];
	memset(history, 0, sizeof(*history));
	history->cycles = TIM2->CNT;
	history->diff_cycles = history->cycles - offset_count;
	history->operation = operation;
	history->preempted_program_pointer = original_program_pointer;
	history->is_in = 0;
	history->epnum = index;
	history->usb_buff = data_available;
	history->dma_pos_at_usb = dma_pos;
}

void TRACING_add_feedback(uint8_t index,
                          int32_t diff_usb,
                          int32_t expected_buff,
                          int32_t adj_usb_buff,
                          int32_t usb_buff,
                          int32_t dma_pos_at_usb,
                          int32_t buffer_processed_flag,
                          int32_t dma_isr_flag,
                          int32_t available_data_for_processing,
                          int32_t usb_available_lock,
                          const char* operation) {
	struct TRACING_Instance* data;

	data = &tracing;

	uint32_t history_index = TRACING_getNewIndex();
	struct history_data* history = &data->history[history_index];
	memset(history, 0, sizeof(*history));
	history->cycles = TIM2->CNT;
	history->diff_cycles = history->cycles - offset_count;
	history->operation = operation;
	history->preempted_program_pointer = original_program_pointer;

	history->epnum = index;
	history->diff_usb = diff_usb;
	history->expected_buff = expected_buff;
	history->adj_usb_buff = adj_usb_buff;
	history->usb_buff = usb_buff;
	history->dma_pos_at_usb = dma_pos_at_usb;
	history->buffer_processed_flag = buffer_processed_flag;
	history->dma_isr_flag = dma_isr_flag;
	history->available_data_for_processing = available_data_for_processing;
	history->usb_available_lock = usb_available_lock;
}

#endif

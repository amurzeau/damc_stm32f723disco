#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENABLE_TRACING

#ifdef ENABLE_TRACING
extern uint32_t offset_count;

void TRACING_init();
void TRACING_reset();
// #define TRACING_add(...)
void TRACING_add(bool in_ep, uint32_t epnum, const char* operation);
void TRACING_add_buffer(uint8_t index, uint32_t data_available, int32_t dma_pos, const char* operation);
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
                          const char* operation);
void TRACING_gpio_set(bool value);
#else
#define TRACING_init()
#define TRACING_reset()
#define TRACING_add(...)
#define TRACING_add_buffer(...)
#define TRACING_gpio_set(...)
#endif

#ifdef __cplusplus
}
#endif

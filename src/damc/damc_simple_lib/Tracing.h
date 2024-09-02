#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// #define ENABLE_TRACING

#ifdef ENABLE_TRACING
extern uint32_t offset_count;

void TRACING_add(bool in_ep, uint8_t epnum, const char* operation);
#else
#define TRACING_add(...)
#endif

#ifdef __cplusplus
}
#endif
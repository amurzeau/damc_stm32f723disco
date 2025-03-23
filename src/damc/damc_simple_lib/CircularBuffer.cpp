#include "CircularBuffer.h"

#include PLATFORM_HEADER

void circularBufferCleanDataCache(volatile void* addr, int32_t dsize) {
	SCB_CleanDCache_by_Addr(addr, dsize);
}

void circularBufferInvalidateDataCache(volatile void* addr, int32_t dsize) {
	SCB_InvalidateDCache_by_Addr(addr, dsize);
}

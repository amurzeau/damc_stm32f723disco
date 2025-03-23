#include "CircularBuffer.h"

#include PLATFORM_HEADER

void circularBufferCleanDataCache(uint32_t* addr, int32_t dsize) {
	SCB_CleanDCache_by_Addr(addr, dsize);
}

void circularBufferInvalidateDataCache(uint32_t* addr, int32_t dsize) {
	SCB_InvalidateDCache_by_Addr(addr, dsize);
}

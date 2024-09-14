#pragma once
#include <stdint.h>
#include <stm32f7xx.h>
#include <stm32f7xx_hal_sai.h>

extern "C" {
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
}

class CodecSTM32F723EDiscoInit {
public:
	void init(bool slaveSAI);
	void init_after_clock_enabled();

	void init_clock();
	void init_sai(bool slaveSAI);
	void init_codec();

	void startTxDMA(void* buffer, size_t size);
	void startRxDMA(void* buffer, size_t size);
	uint16_t getTxRemainingCount();
	uint16_t getRxRemainingCount();

private:
};
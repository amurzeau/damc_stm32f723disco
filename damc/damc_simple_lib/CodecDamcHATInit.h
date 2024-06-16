#pragma once
#include <stdint.h>
#include <stm32f7xx.h>
#include <stm32f7xx_hal_dma.h>
#include <stm32f7xx_hal_i2c.h>
#include <stm32f7xx_hal_sai.h>
#include <stm32f7xx_hal_tim.h>

extern "C" {
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
}

class CodecDamcHATInit {
public:
	void init_i2c();
	bool isAvailable();

	void init();
	void init_sai();
	void init_codec();

	void startTxDMA(void* buffer, size_t nframes);
	void startRxDMA(void* buffer, size_t nframes);
	uint16_t getTxRemainingCount();
	uint16_t getRxRemainingCount();

	void setMicBias(bool enable);

private:
	void writeI2c(uint8_t address, uint8_t value);
	uint8_t readI2c(uint8_t address);
	void setReset(bool value);
	void setTpaEn(bool value);

	friend void DMA2_Stream3_IRQHandler(void);
	friend void DMA2_Stream5_IRQHandler(void);

private:
	I2C_HandleTypeDef hi2c;
	TIM_HandleTypeDef htim;
	SAI_HandleTypeDef hsai_tx;
	SAI_HandleTypeDef hsai_rx;
	DMA_HandleTypeDef hdma_tx;
	DMA_HandleTypeDef hdma_rx;
};
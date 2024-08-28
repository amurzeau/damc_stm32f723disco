#include "CodecDamcHATInit.h"
#include "CodecAudio.h"
#include <stdlib.h>
#include <stm32f723e_discovery.h>
#include <stm32f7xx_hal_gpio.h>

static CodecDamcHATInit* codec = nullptr;

extern "C" {
DMA_HandleTypeDef* hdma2_stream3;
DMA_HandleTypeDef* hdma2_stream5;
}

bool CodecDamcHATInit::isAvailable() {
	// Reset Audio Codec
	setReset(false);
	HAL_Delay(2);  // Delay for 2mS for reset of codec
	setReset(true);
	HAL_Delay(2);  // Delay for 2mS before first write

	// Check if available
	return HAL_I2C_IsDeviceReady(&hi2c, 0x30, 1, 1000) == HAL_OK;
}

void CodecDamcHATInit::init_i2c() {
	__HAL_RCC_I2C2_CLK_ENABLE();
	hi2c.Instance = I2C2;
	hi2c.Init.Timing = DISCOVERY_I2Cx_TIMING;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	/* Init the I2C */
	HAL_I2C_Init(&hi2c);
}

void CodecDamcHATInit::init() {
	codec = this;

	// Configure for 13.5MHz on 54Mhz TIM5_CH2 from 27Mhz APB1 (timers have a frequency of APB x2).
	// Prescaler divide by 1: 54Mhz.
	// Counter is reset after reaching 1 (period = 1), this behavior divide the frequency by 2: 27Mhz.
	// Each time the timer counter is reset, the TIM5_CH2 is toggled, so for each timer reset we have a
	// half-period: 13.5Mhz.
	__HAL_RCC_TIM5_CLK_ENABLE();

	TIM_OC_InitTypeDef sOcConfig = {0};

	htim.Instance = TIM5;
	htim.Init.Prescaler = 0;  // /1 => 13.5Mhz
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = 1;  // Will do /2 as the counter will counter from 0 to 1 to 0, etc...
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_OC_Init(&htim) != HAL_OK) {
	}

	// Will divide by 2 as at each timer reset we toggle the output clock, effectively doing only a half period
	sOcConfig.OCMode = TIM_OCMODE_TOGGLE;
	sOcConfig.Pulse = 0;
	sOcConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	HAL_TIM_OC_ConfigChannel(&htim, &sOcConfig, TIM_CHANNEL_2);

	HAL_TIM_OC_Start(&htim, TIM_CHANNEL_2);

	init_sai();
	init_codec();

	HAL_Delay(1000);
	setTpaEn(true);
}

void CodecDamcHATInit::init_sai() {
	__HAL_RCC_SAI1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA SAI1 TX */

	/* Configure the hdma_sai_rx handle parameters */
	hdma_tx.Init.Channel = DMA_CHANNEL_0;
	hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_tx.Init.Mode = DMA_CIRCULAR;
	hdma_tx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_tx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	hdma_tx.Instance = DMA2_Stream3;
	hdma2_stream3 = &hdma_tx;

	/* Associate the DMA handle */
	__HAL_LINKDMA(&hsai_tx, hdmatx, hdma_tx);

	/* Deinitialize the Stream for new transfer */
	HAL_DMA_DeInit(&hdma_tx);

	/* Configure the DMA Stream */
	HAL_DMA_Init(&hdma_tx);

	/* SAI DMA IRQ Channel configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0x0E, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

	/* DMA SAI1 RX */

	/* Configure the hdma_sai_rx handle parameters */
	hdma_rx.Init.Channel = DMA_CHANNEL_0;
	hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_rx.Init.Mode = DMA_CIRCULAR;
	hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_rx.Init.PeriphBurst = DMA_MBURST_SINGLE;

	hdma_rx.Instance = DMA2_Stream5;
	hdma2_stream5 = &hdma_rx;

	/* Associate the DMA handle */
	__HAL_LINKDMA(&hsai_rx, hdmarx, hdma_rx);

	/* Deinitialize the Stream for new transfer */
	HAL_DMA_DeInit(&hdma_rx);

	/* Configure the DMA Stream */
	HAL_DMA_Init(&hdma_rx);

	/* SAI DMA IRQ Channel configuration */
	// No need for IRQ for RX
	// HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0x0E, 0);
	// HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	/* SAI1 */

	/* Initialize the hsai_tx Instance parameter */
	hsai_tx.Instance = SAI1_Block_A;

	/* Disable SAI peripheral to allow access to SAI internal registers */
	__HAL_SAI_DISABLE(&hsai_tx);

	/* Configure SAI_Block_x
	LSBFirst: Disabled
	DataSize: 32 */
	hsai_tx.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_tx.Init.AudioFrequency = 48000;
	hsai_tx.Init.AudioMode = SAI_MODESLAVE_TX;
	hsai_tx.Init.NoDivider = SAI_MASTERDIVIDER_ENABLED;
	hsai_tx.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_tx.Init.DataSize = SAI_DATASIZE_32;
	hsai_tx.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_tx.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_tx.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_tx.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_tx.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
	hsai_tx.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_tx.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_tx.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_tx.Init.Mckdiv = 0;

	/* Configure SAI_Block_x Frame
	Frame Length: 64
	Frame active Length: 32
	FS Definition: Start frame + Channel Side identification
	FS Polarity: FS active Low
	FS Offset: FS asserted one bit before the first bit of slot 0 */
	hsai_tx.FrameInit.FrameLength = 64;
	hsai_tx.FrameInit.ActiveFrameLength = 32;
	hsai_tx.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
	hsai_tx.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_tx.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

	/* Configure SAI Block_A
	Slot First Bit Offset: 0
	Slot Size  : 32
	Slot Number: 2
	Slot Active: All slot actives */
	hsai_tx.SlotInit.FirstBitOffset = 0;
	hsai_tx.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_tx.SlotInit.SlotNumber = 2;
	hsai_tx.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1;

	HAL_SAI_Init(&hsai_tx);

	/* Configure SAI Block_B */
	hsai_rx.Init = hsai_tx.Init;
	hsai_rx.FrameInit = hsai_tx.FrameInit;
	hsai_rx.SlotInit = hsai_tx.SlotInit;
	hsai_rx.Instance = SAI1_Block_B;
	hsai_rx.Init.AudioMode = SAI_MODESLAVE_RX;
	hsai_rx.Init.Synchro = SAI_SYNCHRONOUS;
	hsai_rx.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;

	__HAL_SAI_DISABLE(&hsai_rx);
	HAL_SAI_Init(&hsai_rx);
}

void CodecDamcHATInit::init_codec() {
	// Reset Audio Codec
	setReset(false);
	HAL_Delay(2);  // Delay for 2mS for reset of codec
	setReset(true);
	HAL_Delay(2);  // Delay for 2mS before first write

	/*
	while(HAL_I2C_IsDeviceReady(&hi2c, 0x30, 1, 1000) != HAL_OK) {
	}
	*/

	writeI2c(0, 0);  // Select page 0

	// PLL_CLK = PLL_CLKIN (13.5Mhz) / P * R * J.D
	// 13.5Mhz / P >= 10Mhz and <= 20Mhz
	// P = 1
	// R = 1
	// J.D = 7.2818
	// PLL_CLK = 98.3043Mhz
	// CODEC_CLKIN = 98.3043Mhz
	// Ratio: 4

	// PLL clock of 98.304 MHz derrived from a 13.5Mhz MCLK input
	writeI2c(7, 0x0B);        // Clock Setting Register 4, PLL D Values (MSB) D=0.2818
	writeI2c(8, 0x02);        // Clock Setting Register 5, PLL D Values (LSB) D=0.2818
	writeI2c(6, 7);           // Clock Setting Register 3, PLL J Values J=7
	writeI2c(5, 0b10010001);  // Clock Setting Register 2, PLL P and R Values P=1, R=1
	writeI2c(4, 0b00000011);  // Clock Setting Register 1, PLL Mux PLL_CLKIN = MCLK = 13.5Mhz

	HAL_Delay(10);  // Delay for 10mS to wait PLL

	writeI2c(11, (1u << 7) | 4u);  // Clock Setting Register 6, NDAC = 4 CODEC_CLKIN=98.304MHz, DAC_CLK=24.576MHz
	// NADC not used, ADC_CLK = DAC_CLK
	// writeI2c(18, (1u << 7) | 4u);  // Clock Setting Register 8, NADC = 4 CODEC_CLKIN=98.304MHz, ADC_CLK=24.576MHz
	writeI2c(18, 4u);

	writeI2c(12, (1u << 7) | 4u);  // Clock Setting Register 7, MDAC = 4 DAC_CLK=24.576MHz, DAC_FS=6.144MHz
	// MADC not used, ADC_MOD_CLK = DAC_MOD_CLK
	// writeI2c(19, (1u << 7) | 4u);  // Clock Setting Register 9, MADC = 4 In=24.576MHz, Out=6.144MHz
	writeI2c(19, 4u);

	// DAC and ADC Clocks set to 48kHz fs
	writeI2c(13, 0);    // DAC OSR Setting Register 1, MSB DOSR = 128 In=~6.144MHz, Out=48kHz
	writeI2c(14, 128);  // DAC OSR Setting Register 2, LSB

	writeI2c(20, 128);  // ADC Oversampling (AOSR) AOSR = 128 In=~6.144MHz, Out=48kHz

	writeI2c(25, 7);              //  Clock Setting Register 10, Multiplexers, CLKOUT = ADC_MOD_CLK
	writeI2c(26, (1u << 7) | 1);  //  Clock Setting Register 11, CLKOUT M divider value = 1

	writeI2c(27, 0b00111100);      // Audio Interface Set 1 WCLK & BCLK out, 32 bit I2S
	writeI2c(28, 0b00000000);      // Audio Interface Set 2 Data Offset = 0 BCLKs
	writeI2c(29, 0b00000000);      // Audio Interface Set 3 BDIV_CLKIN = DAC_CLK = 24.576Mhz
	writeI2c(30, (1u << 7) | 8u);  // Clock Setting Register 12, BCLK = BDIV_CLKIN/8 = 3.072Mhz (2x 32bits slots)
	writeI2c(33, 0b00000000);      // Audio Interface Set 6 Use primary SAI interface

	writeI2c(48, 0b10000000);  // INT1 Interrupt Control Register Headset insertion event trigger INT1 interrupt
	writeI2c(52, 0b00010100);  // GPIO/MFP5 Control Register GPIO/MFP5 is INT1
	writeI2c(55, 0b00000110);  //  MISO/MFP4 Function Control Register is CLKOUT

	// Processing block
	writeI2c(60, 1);  // DAC Signal Processing Block Control Register, PRB_P1
	writeI2c(61, 1);  // ADC Signal Processing Block Control Register, PRB_R1

	// Biquad high pass filter on ADC
	int32_t b0 = 0x7ffada00;
	int32_t b1 = 0x80052600;
	int32_t a1 = 0x7ff5b500;
	uint8_t reg = 24;

	// Left First order IIR
	if(1) {
		writeI2c(0, 8);  // Select page 8
		writeI2c(reg++, b0 >> 24);
		writeI2c(reg++, b0 >> 16);
		writeI2c(reg++, b0 >> 8);
		writeI2c(reg++, b0);

		writeI2c(reg++, b1 >> 24);
		writeI2c(reg++, b1 >> 16);
		writeI2c(reg++, b1 >> 8);
		writeI2c(reg++, b1);

		writeI2c(reg++, a1 >> 24);
		writeI2c(reg++, a1 >> 16);
		writeI2c(reg++, a1 >> 8);
		writeI2c(reg++, a1);

		// Right First order IIR
		writeI2c(0, 9);  // Select page 9
		reg = 32;
		writeI2c(reg++, b0 >> 24);
		writeI2c(reg++, b0 >> 16);
		writeI2c(reg++, b0 >> 8);
		writeI2c(reg++, b0);

		writeI2c(reg++, b1 >> 24);
		writeI2c(reg++, b1 >> 16);
		writeI2c(reg++, b1 >> 8);
		writeI2c(reg++, b1);

		writeI2c(reg++, a1 >> 24);
		writeI2c(reg++, a1 >> 16);
		writeI2c(reg++, a1 >> 8);
		writeI2c(reg++, a1);
	}

	// Power
	writeI2c(0, 1);           // Select page 1
	writeI2c(2, 0b10100000);  // LDO Control Register, DVDD and AVDD LDO Powered up 1.77V
	writeI2c(1, 0b00001000);  // Power Configuration Register, Disable connection of AVDD with DVDD
	writeI2c(2, 0b10100001);  // LDO Control Register, Enable master analog control

	writeI2c(10, 0b00000001);   // Common Mode Control CM 0.9V LDOIN 1.8-3.6V
	writeI2c(61, 0b00000000);   // ADC Power Tune Configuration Register, PTM_R4
	writeI2c(3, 0b00000000);    // Left DAC is Class AB and PTM_P3/P4
	writeI2c(4, 0b00000000);    // Right DAC is Class AB and PTM_P3/P4
	writeI2c(71, 0x32);  // Analog Input Quick Charging Configuration Register, Analog inputs power up time is 3.1 ms
	writeI2c(123, 0b00000011);  // Reference Power-up Configuration Register, power up in 120ms
	HAL_Delay(100);             // Delay for supply rail powerup

	// Configurations and background noise level with MICPGA +20dB:
	// - MICBIAS @1.7V, @AVDD, common mode 0.9V: -70dB
	// - MICBIAS @2.5V, @AVDD, common mode 0.9V: -62dB
	// - MICBIAS direct, @AVDD, common mode 0.9V: -64dB
	// - MICBIAS direct, @LDOIN, common mode 0.9V: -67dB
	// - MICBIAS @2.075V, @AVDD, common mode 0.75V: -59dB
	// MICBIAS not using direct power supply have very high 10khz noise with no load, capacitor to be removed on MICBIAS
	writeI2c(51, 0b01010000);  // MICBIAS Configuration Register, enable MICBIAS @1.7V
	// Bit 7: reserved
	// Bit 6: ON/off
	// Bit 5-4: 1.25V, 1.7V, 2.5V, power supply
	// Bit 3: AVDD, LDOIN
	// Bit 2-0: reserved
	// writeI2c(51, 0b00000000);  // MICBIAS disabled

	writeI2c(9, 0b00001100);  // Output Driver Power Control LOL LOR on
	HAL_Delay(1000);          // Delay for supply rail powerup

	// DAC/ADC routing
	writeI2c(14, 0b00001000);  // LOL Routing Selection Left DAC
	writeI2c(15, 0b00001000);  // LOR Routing Selection Right DAC

	writeI2c(22, 0b01110101);  // IN1L to HPL Volume Control Register muted
	writeI2c(23, 0b01110101);  // IN1R to HPR Volume Control Register muted
	writeI2c(24, 0b00101000);  // Mixer Amplifier Left Volume Control Register muted
	writeI2c(25, 0b00101000);  // Mixer Amplifier Right Volume Control Register muted

	writeI2c(18, 0b00000000);  // LOL Driver Gain Setting Register, LOL unmuted
	writeI2c(19, 0b00000000);  // LOR Driver Gain Setting Register, LOR unmuted

	if(1) {
		// IN1R to Left MICPGA+
		writeI2c(52, 0b00000001);  // Left MICPGA Positive Terminal Input Routing Configuration Register
		// IN2R (SGND) to Left MICPGA-
		writeI2c(54, 0b00010000);  // Left MICPGA Negative Terminal Input Routing Configuration Register

		// IN2L to Right MICPGA+
		writeI2c(55, 0b00000001);  // Right MICPGA Positive Terminal Input Routing Configuration Register
		// IN1L (SGND) to Right MICPGA-
		writeI2c(57, 0b00010000);  // Right MICPGA Negative Terminal Input Routing Configuration Register
	} else if(0) {
		// IN2L to Left MICPGA+
		writeI2c(52, 0b00010000);  // Left MICPGA Positive Terminal Input Routing Configuration Register
		// CM (GND) to Left MICPGA-
		writeI2c(54, 0b01000000);  // Left MICPGA Negative Terminal Input Routing Configuration Register

		// IN2L to Right MICPGA+
		writeI2c(55, 0b00000001);  // Right MICPGA Positive Terminal Input Routing Configuration Register
		// CM (GND) to Right MICPGA-
		writeI2c(57, 0b01000000);  // Right MICPGA Negative Terminal Input Routing Configuration Register
	} else {
		// IN1L (SGND) to Left MICPGA+
		writeI2c(52, 0b01000000);  // Left MICPGA Positive Terminal Input Routing Configuration Register
		// CM (GND) to Left MICPGA-
		writeI2c(54, 0b01000000);  // Left MICPGA Negative Terminal Input Routing Configuration Register

		// IN2R (SGND) to Right MICPGA+
		writeI2c(55, 0b00010000);  // Right MICPGA Positive Terminal Input Routing Configuration Register
		// CM (GND) to Right MICPGA-
		writeI2c(57, 0b01000000);  // Right MICPGA Negative Terminal Input Routing Configuration Register
	}

	writeI2c(58, 0b00001100);  // Floating Input Configuration Register, IN3 weakly driven

	writeI2c(59, 40);  // Left MICPGA Volume Control Register, Left MICPGA = 20dB
	writeI2c(60, 40);  // Right MICPGA Volume Control Register, Left MICPGA = 20dB

	// Enable ADC/DAC
	writeI2c(0, 0);            // Select page 0
	writeI2c(63, 0b11010100);  // DAC Channel Setup Register 1 DAC Enabled
	writeI2c(64, 0b00000010);  // DAC Channel Setup Register 2 DAC unmuted, volume controlled by Left DAC volume
	writeI2c(65, 0b00000000);  // Left DAC Channel Digital Volume DAC volume 0dB

	writeI2c(68, 0b00000000);  // DRC Control Register 1 DRC (compression) disabled

	writeI2c(81, 0b11000000);  // ADC Channel Setup Register, ADC Enabled
	writeI2c(82, 0b00000000);  // ADC Fine Gain Adjust Register ADC unmuted

	// writeI2c(83, 0b01101000);  // ADC Left volume = -12dB
	// writeI2c(84, 0b01101000);  // ADC Right volume = -12dB

	// Headset detection function
	writeI2c(56, 0b00000000);  // SCLK/MFP3 Function Control Register SCLK/MFP3 is disabled
	// writeI2c(67, 0b10000000);  // Headset Detection Configuration Register Enable headset detection
}

void CodecDamcHATInit::startTxDMA(void* buffer, size_t nframes) {
	/* Update the Media layer and enable it for play */
	HAL_SAI_Transmit_DMA(
	    &hsai_tx,
	    (uint8_t*) buffer,
	    nframes * sizeof(CodecAudio::CodecFrame::headphone) / sizeof(CodecAudio::CodecFrame::headphone[0]));

	// Wait the SAI FIFO to be not empty before starting the SAI
	while((hsai_tx.Instance->SR & SAI_xSR_FLVL) == SAI_FIFOSTATUS_EMPTY) {
	}

	/* Enable SAI peripheral */
	__HAL_SAI_ENABLE(&hsai_tx);
}

void CodecDamcHATInit::startRxDMA(void* buffer, size_t nframes) {
	/* Update the Media layer and enable it for play */
	HAL_SAI_Receive_DMA(
	    &hsai_rx,
	    (uint8_t*) buffer,
	    nframes * sizeof(CodecAudio::CodecFrame::headphone) / sizeof(CodecAudio::CodecFrame::headphone[0]));

	/* Enable SAI peripheral */
	__HAL_SAI_ENABLE(&hsai_rx);
}

uint16_t CodecDamcHATInit::getTxRemainingCount(void) {
	return __HAL_DMA_GET_COUNTER(hsai_tx.hdmatx);
}

uint16_t CodecDamcHATInit::getRxRemainingCount(void) {
	return __HAL_DMA_GET_COUNTER(hsai_rx.hdmarx);
}

void CodecDamcHATInit::writeI2c(uint8_t address, uint8_t value) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c, 0x30, address, 1, &value, 1, 1000);
	if(status != HAL_OK) {
		value = 0;
	}
}

uint8_t CodecDamcHATInit::readI2c(uint8_t address) {
	uint8_t value = 0;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c, 0x30, address, 1, &value, 1, 1000);
	if(status != HAL_OK) {
		return 0;
	}
	return value;
}

void CodecDamcHATInit::setReset(bool value) {
	GPIO_PinState gpio_value = value ? GPIO_PIN_SET : GPIO_PIN_RESET;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, gpio_value);
}

void CodecDamcHATInit::setTpaEn(bool value) {
	GPIO_PinState gpio_value = value ? GPIO_PIN_SET : GPIO_PIN_RESET;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_value);
}

void CodecDamcHATInit::setMicBias(bool enable) {
	// Select page 1
	writeI2c(0, 1);
	if(enable) {
		writeI2c(51, 0b01010000);
	} else {
		writeI2c(51, 0b00010000);
	}
}
#include "CodecSTM32F723EDiscoInit.h"
#include "CodecAudio.h"
#include <stdlib.h>
#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_audio.h>

extern SAI_HandleTypeDef haudio_out_sai;
extern SAI_HandleTypeDef haudio_in_sai;

void CodecSTM32F723EDiscoInit::init(bool slaveSAI) {
	if(!slaveSAI) {
		init_clock();
	}

	init_sai(slaveSAI);
}

void CodecSTM32F723EDiscoInit::init_after_clock_enabled() {
	init_codec();
}

void CodecSTM32F723EDiscoInit::init_clock() {
	RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

	HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);
	if(HSE_VALUE == 25000000U) {
		/* SAI clock config
		PLLSAI_VCO: VCO_344M
		SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 1Mhz * 344/7 = 49.142 Mhz
		SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 49.142/1 = 49.142 Mhz */
		rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
		rcc_ex_clk_init_struct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
		rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 344;
		rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 7;
		rcc_ex_clk_init_struct.PLLI2SDivQ = 1;
	} else {
		/* SAI clock config
		PLLSAI_VCO: VCO_344M
		SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 2Mhz * 172/7 = 49,143 Mhz
		SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 49,143/1 = 49,143 Mhz */
		rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
		rcc_ex_clk_init_struct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
		rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 172;
		rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 7;
		rcc_ex_clk_init_struct.PLLI2SDivQ = 1;
	}

	HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
}

void CodecSTM32F723EDiscoInit::init_sai(bool slaveSAI) {
	/* SAI2 */

	/* Initialize the haudio_out_sai Instance parameter */
	haudio_in_sai.Instance = AUDIO_IN_SAIx;
	BSP_AUDIO_IN_MspInit(&haudio_in_sai, NULL);

	haudio_out_sai.Instance = AUDIO_OUT_SAIx;
	BSP_AUDIO_OUT_MspInit(&haudio_out_sai, NULL);

	/* Disable SAI peripheral to allow access to SAI internal registers */
	__HAL_SAI_DISABLE(&haudio_out_sai);

	/* Configure SAI_Block_x
	LSBFirst: Disabled
	DataSize: 16 */
	haudio_out_sai.Init.AudioFrequency = 48000;
	haudio_out_sai.Init.AudioMode = SAI_MODEMASTER_TX;
	haudio_out_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	haudio_out_sai.Init.Protocol = SAI_FREE_PROTOCOL;
	haudio_out_sai.Init.DataSize = SAI_DATASIZE_32;
	haudio_out_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
	haudio_out_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	haudio_out_sai.Init.Synchro = slaveSAI ? SAI_SYNCHRONOUS : SAI_ASYNCHRONOUS;
	haudio_out_sai.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	haudio_out_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	haudio_out_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
	haudio_out_sai.Init.Mckdiv = 0;
	haudio_out_sai.Init.MonoStereoMode = SAI_STEREOMODE;
	haudio_out_sai.Init.CompandingMode = SAI_NOCOMPANDING;
	haudio_out_sai.Init.TriState = SAI_OUTPUT_NOTRELEASED;

	/* Configure SAI_Block_x Frame
	Frame Length: 64
	Frame active Length: 32
	FS Definition: Start frame + Channel Side identification
	FS Polarity: FS active Low
	FS Offset: FS asserted one bit before the first bit of slot 0 */
	haudio_out_sai.FrameInit.FrameLength = 128;
	haudio_out_sai.FrameInit.ActiveFrameLength = 64;
	haudio_out_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
	haudio_out_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	haudio_out_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
	haudio_in_sai.FrameInit = haudio_out_sai.FrameInit;

	/* Configure SAI Block_x Slot
	Slot First Bit Offset: 0
	Slot Size  : 16
	Slot Number: 4
	Slot Active: All slot actives */
	haudio_out_sai.SlotInit.FirstBitOffset = 0;
	haudio_out_sai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	haudio_out_sai.SlotInit.SlotNumber = 4;
	haudio_out_sai.SlotInit.SlotActive = CODEC_AUDIOFRAME_SLOT_02;

	HAL_SAI_Init(&haudio_out_sai);

	/* Configure SAI Block_B */
	__HAL_SAI_DISABLE(&haudio_in_sai);

	haudio_in_sai.Init = haudio_out_sai.Init;
	haudio_in_sai.FrameInit = haudio_out_sai.FrameInit;
	haudio_in_sai.SlotInit = haudio_out_sai.SlotInit;
	haudio_in_sai.Init.AudioMode = SAI_MODESLAVE_RX;
	haudio_in_sai.Init.Synchro = SAI_SYNCHRONOUS;
	haudio_in_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;

	HAL_SAI_Init(&haudio_in_sai);

	/* Enable SAI peripheral */
	__HAL_SAI_ENABLE(&haudio_in_sai);
	__HAL_SAI_ENABLE(&haudio_out_sai);
}

void CodecSTM32F723EDiscoInit::init_codec() {
	uint32_t deviceid = 0x00;
	deviceid = wm8994_drv.ReadID(AUDIO_I2C_ADDRESS);

	if((deviceid) == WM8994_ID) {
		/* Reset the Codec Registers */
		wm8994_drv.Reset(AUDIO_I2C_ADDRESS);
		/* Initialize the audio driver structure */
		// 91 gives 0dB HPOUT1L_VOL
		// 80 gives 0dB AIF1ADC1L_VOL
		wm8994_drv.Init(AUDIO_I2C_ADDRESS, INPUT_DEVICE_INPUT_LINE_1 | OUTPUT_DEVICE_HEADPHONE, 91, 80, 48000);
	}
}

void CodecSTM32F723EDiscoInit::startTxDMA(void* buffer, size_t nframes) {
	// Unmute
	wm8994_drv.Play(AUDIO_I2C_ADDRESS, nullptr, 0);
	/* Update the Media layer and enable it for play */
	HAL_SAI_Transmit_DMA(
	    &haudio_out_sai,
	    (uint8_t*) buffer,
	    nframes * sizeof(CodecAudio::CodecFrame::headphone) / sizeof(CodecAudio::CodecFrame::headphone[0]));
}

void CodecSTM32F723EDiscoInit::startRxDMA(void* buffer, size_t nframes) {
	/* Update the Media layer and enable it for play */
	HAL_SAI_Receive_DMA(
	    &haudio_in_sai,
	    (uint8_t*) buffer,
	    nframes * sizeof(CodecAudio::CodecFrame::headphone) / sizeof(CodecAudio::CodecFrame::headphone[0]));
}

uint16_t CodecSTM32F723EDiscoInit::getTxRemainingCount(void) {
	return __HAL_DMA_GET_COUNTER(haudio_out_sai.hdmatx);
}

uint16_t CodecSTM32F723EDiscoInit::getRxRemainingCount(void) {
	return __HAL_DMA_GET_COUNTER(haudio_in_sai.hdmarx);
}

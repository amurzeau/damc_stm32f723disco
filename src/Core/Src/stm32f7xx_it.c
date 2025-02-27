/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AudioCApi.h"
#include "stm32f723e_discovery_audio.h"
#include "stm32f7xx_hal_qspi.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx_hal_dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef* hdma2_stream3;
extern DMA_HandleTypeDef* hdma2_stream5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  return;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */

uint32_t original_program_pointer;
void OTG_HS_IRQHandler_real(uint32_t original_msp)
{
  // Stack when interrupt happen
  // 0x2FF4 preempted task's stack data
  // 0x2FF0 xPSR
  // 0x2FEC PC
  // 0x2FE8 LR
  // 0x2FE4 R12
  // 0x2FE0 R3
  // 0x2FDC R2
  // 0x2FD8 R1
  // 0x2FD4 R0  <--- Thread Stack Pointer when interrupt happened (R13)
  const uint32_t *sp = (const uint32_t *)original_msp;
  original_program_pointer = sp[6];
  __DMB();

  /* USER CODE BEGIN OTG_HS_IRQn 0 */
  DAMC_beginMeasure(TMI_UsbInterrupt);
  SYS_LD_USER2_GPIO_Port->BSRR = SYS_LD_USER2_Pin << 16;

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */
  SYS_LD_USER2_GPIO_Port->BSRR = SYS_LD_USER2_Pin;
  DAMC_endMeasure(TMI_UsbInterrupt);

  __DMB();
  original_program_pointer = 0;

  /* USER CODE END OTG_HS_IRQn 1 */
}

__attribute__((naked)) void OTG_HS_IRQHandler(void)
{
  // Find the original stack pointer in a naked function to ensure the compiler doesn't use the stack for local variables.
  // The real ISR handle is then called with the stack pointer value as argument
  asm("tst lr, #4\n"                                          // Test for MSP or PSP
      "ite eq\n"                                              // If equal
      "mrseq r0, msp\n"                                       //  r0 = msp
      "mrsne r0, psp\n"                                       // else r0 = psp
      "b %[OTG_HS_IRQHandler_real]\n"                         // Call real handler
      :                                                       // output
      : [OTG_HS_IRQHandler_real] "i"(OTG_HS_IRQHandler_real)  // input
      : "r0"                                                  // clobber
  );
}


/* USER CODE BEGIN 1 */


extern SAI_HandleTypeDef         haudio_out_sai;
extern SAI_HandleTypeDef         haudio_in_sai;

void AUDIO_OUT_SAIx_DMAx_IRQHandler(void)
{
  // Only monitor cpu usage on TX DMA interrupt
  DAMC_beginMeasure(TMI_AudioProcessing);
  // Reset buffer processed flags before the DMA interrupt is cleared.
  DAMC_resetBufferProcessedFlags();
  __DMB();
  HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
  DAMC_endMeasure(TMI_AudioProcessing);
}

void AUDIO_IN_SAIx_DMAx_IRQHandler(void)
{
  HAL_DMA_IRQHandler(haudio_in_sai.hdmarx);
}

void AUDIO_IN_INT_IRQHandler(void)
{
  DAMC_beginMeasure(TMI_OtherIRQ);
  HAL_GPIO_EXTI_IRQHandler(AUDIO_IN_INT_GPIO_PIN);
  DAMC_endMeasure(TMI_OtherIRQ);
}

/* HAT SAI DMA2 IRQs */
void DMA2_Stream3_IRQHandler(void) {
  // Only monitor cpu usage on TX DMA interrupt
  DAMC_beginMeasure(TMI_AudioProcessing);
  // Reset buffer processed flags before the DMA interrupt is cleared.
  DAMC_resetBufferProcessedFlags();
  __DMB();
  HAL_DMA_IRQHandler(hdma2_stream3);
  DAMC_endMeasure(TMI_AudioProcessing);
}

void DMA2_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(hdma2_stream5);
}

extern QSPI_HandleTypeDef QSPIHandle;
void QUADSPI_IRQHandler(void)
{
  DAMC_beginMeasure(TMI_OtherIRQ);
  HAL_QSPI_IRQHandler(&QSPIHandle);
  DAMC_endMeasure(TMI_OtherIRQ);
}

// TS_INT IRQ
void EXTI9_5_IRQHandler() {
  DAMC_beginMeasure(TMI_OtherIRQ);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  DAMC_endMeasure(TMI_OtherIRQ);
}

/* USER CODE END 1 */

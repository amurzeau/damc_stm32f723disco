/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32n6xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32n6xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AudioCApi.h"
#include <stm32n6570_discovery_ts.h>
#include <stm32n6570_discovery_audio.h>
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

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
  * @brief This function handles Secure fault.
  */
void SecureFault_Handler(void)
{
  /* USER CODE BEGIN SecureFault_IRQn 0 */

  /* USER CODE END SecureFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_SecureFault_IRQn 0 */
    /* USER CODE END W1_SecureFault_IRQn 0 */
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
/* STM32N6xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32n6xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Serial Audio Interface 1 block A interrupt.
  */
void SAI1_A_IRQHandler(void)
{
  /* USER CODE BEGIN SAI1_A_IRQn 0 */

  /* USER CODE END SAI1_A_IRQn 0 */
  /* USER CODE BEGIN SAI1_A_IRQn 1 */

  /* USER CODE END SAI1_A_IRQn 1 */
}

/**
  * @brief This function handles Serial Audio Interface 1 block B interrupt.
  */
void SAI1_B_IRQHandler(void)
{
  /* USER CODE BEGIN SAI1_B_IRQn 0 */

  /* USER CODE END SAI1_B_IRQn 0 */
  /* USER CODE BEGIN SAI1_B_IRQn 1 */

  /* USER CODE END SAI1_B_IRQn 1 */
}

/* USER CODE BEGIN 1 */

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

uint32_t original_program_pointer;
void USB1_OTG_HS_IRQHandler_real(uint32_t original_msp)
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

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */
  DAMC_endMeasure(TMI_UsbInterrupt);

  __DMB();
  original_program_pointer = 0;

  /* USER CODE END OTG_HS_IRQn 1 */
}

__attribute__((naked)) void USB1_OTG_HS_IRQHandler(void)
{
  // Find the original stack pointer in a naked function to ensure the compiler doesn't use the stack for local variables.
  // The real ISR handle is then called with the stack pointer value as argument
  asm("tst lr, #4\n"                                                    // Test for MSP or PSP
      "ite eq\n"                                                        // If equal
      "mrseq r0, msp\n"                                                 //  r0 = msp
      "mrsne r0, psp\n"                                                 // else r0 = psp
      "b %[USB1_OTG_HS_IRQHandler_real]\n"                              // Call real handler
      :                                                                 // output
      : [USB1_OTG_HS_IRQHandler_real] "i"(USB1_OTG_HS_IRQHandler_real)  // input
      : "r0"                                                            // clobber
  );
}


void GPDMA1_Channel2_IRQHandler(void)
{
  // TX
  // Only monitor cpu usage on TX DMA interrupt
  DAMC_beginMeasure(TMI_AudioProcessing);
  // Reset buffer processed flags before the DMA interrupt is cleared.
  DAMC_resetBufferProcessedFlags();
  __DMB();
  BSP_AUDIO_OUT_IRQHandler(0, 0);
  DAMC_endMeasure(TMI_AudioProcessing);
}

void GPDMA1_Channel1_IRQHandler(void)
{
  // RX
  BSP_AUDIO_IN_IRQHandler(0, 0);
}

// TS_INT IRQ
void EXTI4_IRQHandler()
{
  DAMC_beginMeasure(TMI_OtherIRQ);
  BSP_TS_IRQHandler(0);
  DAMC_endMeasure(TMI_OtherIRQ);
}

/* USER CODE END 1 */

/**
  ******************************************************************************
  * @file    stm32f723e_discovery_psram.c
  * @author  MCD Application Team
  * @brief   This file includes the PSRAM driver for the IS61WV51216BLL-10MLI memory 
  *          device mounted on STM32F723E-DISCOVERY boards.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  How To use this driver:
  -----------------------
   - This driver is used to drive the IS61WV51216BLL-10M PSRAM external memory mounted
     on STM32F723E discovery board.
   - This driver does not need a specific component driver for the PSRAM device
     to be included with.

  Driver description:
  ------------------
  + Initialization steps:
     o Initialize the PSRAM external memory using the BSP_PSRAM_Init() function. This 
       function includes the MSP layer hardware resources initialization and the
       FMC controller configuration to interface with the external PSRAM memory.
  
  + PSRAM read/write operations
     o PSRAM external memory can be accessed with read/write operations once it is
       initialized.
       Read/write operation can be performed with AHB access using the functions
       BSP_PSRAM_ReadData()/BSP_PSRAM_WriteData(), or by DMA transfer using the functions
       BSP_PSRAM_ReadData_DMA()/BSP_PSRAM_WriteData_DMA().
     o The AHB access is performed with 16-bit width transaction, the DMA transfer
       configuration is fixed at single (no burst) halfword transfer 
       (see the PSRAM_MspInit() static function).
     o User can implement his own functions for read/write access with his desired 
       configurations.
     o If interrupt mode is used for DMA transfer, the function BSP_PSRAM_DMA_IRQHandler()
       is called in IRQ handler file, to serve the generated interrupt once the DMA 
       transfer is complete.
  @endverbatim
  ******************************************************************************
  */

/* Dependencies
- stm32f7xx_hal_sram.c
- stm32f7xx_hal_gpio.c
- stm32f7xx_hal_dma.c
- stm32f7xx_hal_cortex.c
- stm32f7xx_hal_rcc_ex.h
EndDependencies */

/* Includes ------------------------------------------------------------------*/
#include "stm32f723e_discovery_psram.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F723E_DISCOVERY
  * @{
  */ 
  
/** @defgroup STM32F723E_DISCOVERY_PSRAM STM32F723E-DISCOVERY PSRAM
  * @{
  */ 

/** @defgroup STM32F723E_DISCOVERY_PSRAM_Private_Types_Definitions PSRAM Private Types Definitions
  * @{
  */ 
/**
  * @}
  */

/** @defgroup STM32F723E_DISCOVERY_PSRAM_Private_Defines  PSRAM Private Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32F723E_DISCOVERY_PSRAM_Private_Macros PSRAM Private Macros
  * @{
  */  
/**
  * @}
  */

/** @defgroup STM32F723E_DISCOVERY_PSRAM_Private_Variables PSRAM Private Variables
  * @{
  */       
SRAM_HandleTypeDef psramHandle;

/**
  * @}
  */ 

/** @defgroup STM32F723E_DISCOVERY_PSRAM_Private_Function_Prototypes PSRAM Private Function Prototypes
  * @{
  */ 
/**
  * @}
  */
    
/** @defgroup STM32F723E_DISCOVERY_PSRAM_Private_Functions PSRAM Private Functions
  * @{
  */ 

/**
  * @brief  Initializes the PSRAM device.
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_Init(void)
{
  static FMC_NORSRAM_TimingTypeDef ReadTiming;
  static FMC_NORSRAM_TimingTypeDef WriteTiming;
  static uint8_t psram_status = PSRAM_ERROR;
  
  /* PSRAM device configuration */
  psramHandle.Instance = FMC_NORSRAM_DEVICE;
  psramHandle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  
  /* PSRAM device configuration */
  /* Timing configuration derived from system clock (up to 216Mhz)
     for 108Mhz as PSRAM clock frequency */
  ReadTiming.AddressSetupTime      = 4;  // ADDSET+ADDHLD: +2 HCLK margin, only 13 is stricly required for ADDSET+ADDHLD+DATAST for 60ns access, but sometimes memtester still fails on read
  ReadTiming.AddressHoldTime       = 1;  // ADDHLD, must be at least 1
  ReadTiming.DataSetupTime         = 10; // DATAST: Minimum 60ns read total duration (including ADDSET)
                                     // For reads, it must be at least 25ns
  ReadTiming.BusTurnAroundDuration = 4;  // Bus turnaround after read, at least 1 so chip select is never held for too long, 20ns before HighZ from NOE released (1+4 HCLK)
  ReadTiming.CLKDivision           = 2;  // Don't care in asynchronous mode
  ReadTiming.DataLatency           = 0;  // Don't care in asynchronous mode
  ReadTiming.AccessMode            = FMC_ACCESS_MODE_D; // Use mode D to avoid long period with chip select active

  
  WriteTiming.AddressSetupTime      = 1;  // ADDSET+ADDHLD: Minimum 20ns to HighZ from possible previous read exluding 3 HCLK (13.9ns) from bus turn around
  WriteTiming.AddressHoldTime       = 1;  // ADDHLD, must be at least 1
  WriteTiming.DataSetupTime         = 10; // DATAST: Minimum 55ns write total duration (including ADDSET)
                                          // For writes, it must be at least 45ns
  WriteTiming.BusTurnAroundDuration = 1;  // Bus turnaround after read, at least 1 so chip select is never held for too long
  WriteTiming.CLKDivision           = 2;  // Don't care in asynchronous mode
  WriteTiming.DataLatency           = 0;  // Don't care in asynchronous mode
  WriteTiming.AccessMode            = FMC_ACCESS_MODE_D; // Use mode D to avoid long period with chip select active
//   Timing.AddressSetupTime      = 9;
//   Timing.AddressHoldTime       = 2; // Don't care
//   Timing.DataSetupTime         = 6;
//   Timing.BusTurnAroundDuration = 1;
//   Timing.CLKDivision           = 2;
//   Timing.DataLatency           = 2;
//   Timing.AccessMode            = FMC_ACCESS_MODE_A;
  
  psramHandle.Init.NSBank             = FMC_NORSRAM_BANK1;
  psramHandle.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
  psramHandle.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
  psramHandle.Init.MemoryDataWidth    = PSRAM_MEMORY_WIDTH;
  psramHandle.Init.BurstAccessMode    = PSRAM_BURSTACCESS;
  psramHandle.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  psramHandle.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
  psramHandle.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
  psramHandle.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
  psramHandle.Init.ExtendedMode       = FMC_EXTENDED_MODE_ENABLE; // Use extended mode for mode D
  psramHandle.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  psramHandle.Init.WriteBurst         = PSRAM_WRITEBURST;
  psramHandle.Init.WriteFifo          = FMC_WRITE_FIFO_ENABLE; // Enable Write FIFO to avoid errata
  psramHandle.Init.PageSize           = FMC_PAGE_SIZE_NONE;  
  psramHandle.Init.ContinuousClock    = CONTINUOUSCLOCK_FEATURE;
  
  /* PSRAM controller initialization */
  BSP_PSRAM_MspInit(&psramHandle, NULL); /* __weak function can be rewritten by the application */
  if(HAL_SRAM_Init(&psramHandle, &ReadTiming, &WriteTiming) != HAL_OK)
  {
    psram_status = PSRAM_ERROR;
  }
  else
  {
    psram_status = PSRAM_OK;
  }
  return psram_status;
}

/**
  * @brief  DeInitializes the PSRAM device.
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_DeInit(void)
{ 
  static uint8_t psram_status = PSRAM_ERROR;
  /* PSRAM device de-initialization */
  psramHandle.Instance = FMC_NORSRAM_DEVICE;
  psramHandle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;

  if(HAL_SRAM_DeInit(&psramHandle) != HAL_OK)
  {
    psram_status = PSRAM_ERROR;
  }
  else
  {
    psram_status = PSRAM_OK;
  }
  
  /* PSRAM controller de-initialization */
  BSP_PSRAM_MspDeInit(&psramHandle, NULL);
  
  return psram_status;
}

/**
  * @brief  Reads an amount of data from the PSRAM device in polling mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory   
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_ReadData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{ 
  if(HAL_SRAM_Read_16b(&psramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  }
}

/**
  * @brief  Reads an amount of data from the PSRAM device in DMA mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory   
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_ReadData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{
  if(HAL_SRAM_Read_DMA(&psramHandle, (uint32_t *)uwStartAddress, (uint32_t *)pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  }
}

/**
  * @brief  Writes an amount of data from the PSRAM device in polling mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory   
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_WriteData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize) 
{ 
  if(HAL_SRAM_Write_16b(&psramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  }
}

/**
  * @brief  Writes an amount of data from the PSRAM device in DMA mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory   
  * @retval PSRAM status
  */
uint8_t BSP_PSRAM_WriteData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize) 
{
  if(HAL_SRAM_Write_DMA(&psramHandle, (uint32_t *)uwStartAddress, (uint32_t *)pData, uwDataSize) != HAL_OK)
  {
    return PSRAM_ERROR;
  }
  else
  {
    return PSRAM_OK;
  } 
}

/**
  * @brief  Initializes PSRAM MSP.
  * @param  hsram: PSRAM handle
  * @param  Params  
  * @retval None
  */
__weak void BSP_PSRAM_MspInit(SRAM_HandleTypeDef  *hsram, void *Params)
{  
  static DMA_HandleTypeDef dma_handle;
  GPIO_InitTypeDef gpio_init_structure;
  
  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();
  
  /* Enable chosen DMAx clock */
  __PSRAM_DMAx_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  
  /* Common GPIO configuration */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_FMC;

  /* GPIOD configuration: GPIO_PIN_7 is  FMC_NE1 , GPIO_PIN_11 ans GPIO_PIN_12 are PSRAM_A16 and PSRAM_A17 */
  /* LCD_PSRAM_D2, LCD_PSRAM_D3, LCD_PSRAM_NOE, LCD_PSRAM_NWE, PSRAM_NE1, LCD_PSRAM_D13, 
     LCD_PSRAM_D14, LCD_PSRAM_D15, PSRAM_A16, PSRAM_A17, LCD_PSRAM_D0, LCD_PSRAM_D1 */
  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 |\
                              GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_15;
   
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* GPIOE configuration */  
  /* PSRAM_NBL0, PSRAM_NBL1, LCD_PSRAM_D4, LCD_PSRAM_D5, LCD_PSRAM_D6, LCD_PSRAM_D7, 
     LCD_PSRAM_D8, LCD_PSRAM_D9, LCD_PSRAM_D10, LCD_PSRAM_D11, LCD_PSRAM_D12 */
  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |\
                              GPIO_PIN_12 |GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &gpio_init_structure);
  
  /* GPIOF configuration */  
  /* PSRAM_A0, PSRAM_A1, PSRAM_A2, PSRAM_A3, PSRAM_A4, PSRAM_A5, 
     PSRAM_A6, PSRAM_A7, PSRAM_A8, PSRAM_A9 */
  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |\
                              GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; 
  HAL_GPIO_Init(GPIOF, &gpio_init_structure);

  /* GPIOG configuration */  
  /* PSRAM_A10, PSRAM_A11, PSRAM_A12, PSRAM_A13, PSRAM_A14, PSRAM_A15, LCD_NE */
  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |\
                              GPIO_PIN_5 | GPIO_PIN_9; 
  HAL_GPIO_Init(GPIOG, &gpio_init_structure); 
  
  /* Configure common DMA parameters */
  dma_handle.Init.Channel             = PSRAM_DMAx_CHANNEL;
  dma_handle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
  dma_handle.Init.PeriphInc           = DMA_PINC_ENABLE;
  dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  dma_handle.Init.Mode                = DMA_NORMAL;
  dma_handle.Init.Priority            = DMA_PRIORITY_HIGH;
  dma_handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  dma_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
  dma_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  
  dma_handle.Instance = PSRAM_DMAx_STREAM;
  
   /* Associate the DMA handle */
  __HAL_LINKDMA(hsram, hdma, dma_handle);
  
  /* Deinitialize the Stream for new transfer */
  HAL_DMA_DeInit(&dma_handle);
  
  /* Configure the DMA Stream */
  HAL_DMA_Init(&dma_handle);
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(PSRAM_DMAx_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(PSRAM_DMAx_IRQn);   
}


/**
  * @brief  DeInitializes SRAM MSP.
  * @param  hsram: SRAM handle
  * @param  Params  
  * @retval None
  */
__weak void BSP_PSRAM_MspDeInit(SRAM_HandleTypeDef  *hsram, void *Params)
{  
    static DMA_HandleTypeDef dma_handle;
  
    /* Disable NVIC configuration for DMA interrupt */
    HAL_NVIC_DisableIRQ(PSRAM_DMAx_IRQn);

    /* Deinitialize the stream for new transfer */
    dma_handle.Instance = PSRAM_DMAx_STREAM;
    HAL_DMA_DeInit(&dma_handle);

    /* GPIO pins clock, FMC clock and DMA clock can be shut down in the applications
       by surcharging this __weak function */ 
}
/**
  * @}
  */  
  
/**
  * @}
  */ 
  
/**
  * @}
  */ 
  
/**
  * @}
  */ 


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_audio.h"
#include "AudioCApi.h"
#include "usbd_cdc_if.h"
#include "memtester.h"
#include "stm32f723e_discovery_psram.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

static void SystemClock_Config_24MhzHSE(void);
static void PeriphCommonClock_Config_24MhzHSE(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  MPU_Config(1);

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  if(HSE_VALUE == 25000000U) {

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  } else {
	  /* Configure the system clock for 24Mhz HSE */
	  SystemClock_Config_24MhzHSE();

	/* Configure the peripherals common clocks for 24Mhz HSE */
	  PeriphCommonClock_Config_24MhzHSE();
  }

  MX_GPIO_Init();
  // Initialize PSRAM as external RAM before DAMC_init as DAMC will use the heap on the PSRAM
  BSP_PSRAM_Init();

  // memtester_stm32((void*)0x60000000, 512*1024, 10);


  DAMC_init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  DAMC_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(usb_new_frame_flag && 0) {
		  usb_new_frame_flag = 0;

		  size_t nframes = usb_audio_endpoint_out_data[0].nominal_packet_size / USBD_AUDIO_BYTES_PER_SAMPLE / USBD_AUDIO_CHANNELS;

		  USBD_AUDIO_LoopbackDataTypeDef* data_out[2] = {
			&usb_audio_endpoint_out_data[0],
			&usb_audio_endpoint_out_data[1],
		  };
		  USBD_AUDIO_LoopbackDataTypeDef* data_in[1] = {
			&usb_audio_endpoint_in_data[0]
		  };

		  uint8_t* endpoint_out_buffer[2];
		  uint8_t* endpoint_in_buffer[1];

		  for(size_t i = 0; i < sizeof(data_out)/sizeof(data_out[0]); i++) {
			  endpoint_out_buffer[i] = USBD_AUDIO_GetBufferFromApp(data_out[i]);
		  }
		  for(size_t i = 0; i < sizeof(data_in)/sizeof(data_in[0]); i++) {
			  endpoint_in_buffer[i] = USBD_AUDIO_GetBufferFromApp(data_in[i]);
		  }

		  DAMC_processAudioInterleaved(
				  (const int16_t**)endpoint_out_buffer,
				  sizeof(data_out)/sizeof(data_out[0]),
				  (int16_t**)endpoint_in_buffer,
				  sizeof(data_in)/sizeof(data_in[0]),
				  nframes);

		  for(size_t i = 0; i < sizeof(data_out)/sizeof(data_out[0]); i++) {
			  USBD_AUDIO_ReleaseBufferFromApp(data_out[i]);
		  }
		  for(size_t i = 0; i < sizeof(data_in)/sizeof(data_in[0]); i++) {
			  USBD_AUDIO_ReleaseBufferFromApp(data_in[i]);
		  }
	  }

	  DAMC_mainLoop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 300;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 25;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x6000030D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  // This timer is used to measure time to process data

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, WIFI_RST_Pin|WIFI_GPIO_0_Pin|PMOD_GPIO_0_Pin|USB_OTGFS_PPWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, WIFI_GPIO_2_Pin|WIFI_CH_PD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STMOD_UART4_RXD_s_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, PMOD_SPI2_MOSI_Pin|PMOD_SPI2_MISO_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PMOD_SEL_0_GPIO_Port, PMOD_SEL_0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USB_OTG_FS_ID_Pin|SYS_LD_USER1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, PMOD_GPIO_1_Pin|USB_OTGHS_PPWR_EN_Pin|LCD_BL_Pin
                          |CTP_RST_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USB_OTG_HS_ID_Pin|SYS_LD_USER2_Pin, GPIO_PIN_RESET);


  /* Configure DAMC HAT board */

  /* Configure GPIO pins :
   * PH4: I2C2_SCL
   * PH5: I2C2_SDA
   */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Configure GPIO pins: OUTPUT
   * PB5: AIC_RESET
   * PB0: TPA_EN
   */
  // Set to 0 before configuring
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_0, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pins: INPUT pullup
   * PH6: AIC_INT
   * PH3: JACK2_DETECT
   */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Configure GPIO pins: INPUT pullup
   * PC5: JACK_DETECT
   */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure GPIO pins : PA1: TIM5_CH2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pins : 
   * PE4: SAI1_FS_A
   * PE3: SAI1_SD_B
   * PE6: SAI1_SD_A
   * PE5: SAI1_SCK_A
   */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);





  /*Configure GPIO pins : WIFI_RST_Pin WIFI_GPIO_0_Pin PMOD_GPIO_0_Pin USB_OTGFS_PPWR_EN_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin|WIFI_GPIO_0_Pin|PMOD_GPIO_0_Pin|USB_OTGFS_PPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_TXD_WIFI_RX_Pin */
  GPIO_InitStruct.Pin = UART_TXD_WIFI_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(UART_TXD_WIFI_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_INT_Pin */
  GPIO_InitStruct.Pin = SAI2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SAI2_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WIFI_GPIO_2_Pin WIFI_CH_PD_Pin */
  GPIO_InitStruct.Pin = WIFI_GPIO_2_Pin|WIFI_CH_PD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : STMOD_UART4_RXD_s_Pin */
  GPIO_InitStruct.Pin = STMOD_UART4_RXD_s_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_SPI2_MOSI_Pin PMOD_SPI2_MISO_Pin PI10 */
  GPIO_InitStruct.Pin = PMOD_SPI2_MOSI_Pin|PMOD_SPI2_MISO_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_INT_Pin */
  GPIO_InitStruct.Pin = CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART_RXD_WIFI_TX_Pin */
  GPIO_InitStruct.Pin = UART_RXD_WIFI_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(UART_RXD_WIFI_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_SEL_0_Pin PMOD_GPIO_1_Pin USB_OTGHS_PPWR_EN_Pin
                           LCD_BL_Pin CTP_RST_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = PMOD_SEL_0_Pin|PMOD_GPIO_1_Pin|USB_OTGHS_PPWR_EN_Pin
                          |LCD_BL_Pin|CTP_RST_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_SPI2_SCK_Pin PMOD_SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin|PMOD_SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin SYS_LD_USER1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|SYS_LD_USER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STMOD_UART4_TXD_Pin STMOD_UART4_RXD_Pin */
  GPIO_InitStruct.Pin = STMOD_UART4_TXD_Pin|STMOD_UART4_RXD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_SCL_Pin */
  GPIO_InitStruct.Pin = CTP_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(CTP_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_TE_INT_Pin */
  GPIO_InitStruct.Pin = LCD_TE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_TE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART7_TXD_Pin PMOD_UART7_RXD_Pin PMOD_UART7_CTS_Pin PMOD_UART7_RTS_Pin */
  GPIO_InitStruct.Pin = PMOD_UART7_TXD_Pin|PMOD_UART7_RXD_Pin|PMOD_UART7_CTS_Pin|PMOD_UART7_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_A3_ADC3_IN8_Pin */
  GPIO_InitStruct.Pin = ARD_A3_ADC3_IN8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_A3_ADC3_IN8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTGHS_OVCR_INT_Pin */
  GPIO_InitStruct.Pin = USB_OTGHS_OVCR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTGHS_OVCR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A4_Pin ARD_A5_Pin ARD_A2_Pin */
  GPIO_InitStruct.Pin = ARD_A4_Pin|ARD_A5_Pin|ARD_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STMOD_SPI2_MISOs_Pin STMOD_SPI2_MOSIs_Pin */
  GPIO_InitStruct.Pin = STMOD_SPI2_MISOs_Pin|STMOD_SPI2_MOSIs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D9_TIM12_CH1_Pin */
  GPIO_InitStruct.Pin = ARD_D9_TIM12_CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(ARD_D9_TIM12_CH1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_SDA_Pin */
  GPIO_InitStruct.Pin = CTP_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(CTP_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_USART2_TX_Pin ARD_D0_USART2_RX_Pin */
  GPIO_InitStruct.Pin = ARD_D1_USART2_TX_Pin|ARD_D0_USART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D13_SPI1_SCK_Pin */
  GPIO_InitStruct.Pin = ARD_D13_SPI1_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(ARD_D13_SPI1_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_HS_ID_Pin SYS_LD_USER2_Pin */
  GPIO_InitStruct.Pin = USB_OTG_HS_ID_Pin|SYS_LD_USER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_HS_VBUS_Pin USB_OTGFS_OVCR_INT_Pin PMOD_INT_Pin */
  GPIO_InitStruct.Pin = USB_OTG_HS_VBUS_Pin|USB_OTGFS_OVCR_INT_Pin|PMOD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_RESET_Pin */
  GPIO_InitStruct.Pin = PMOD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PMOD_RESET_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config_24MhzHSE(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config_24MhzHSE(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Configure the MPU attributes for PSRAM mapped
 * @param None
 * @retval None
 */
void MPU_Config(int enable_psram)
{
 MPU_Region_InitTypeDef MPU_InitStruct;

 /* Disable the MPU */
 HAL_MPU_Disable();

 /* Configure the MPU as no access by default, except for peripherals */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0x00;
 MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
 MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER0;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
 // Disable region for peripherals:
 // Each region is 512MB
 // Disabled regions:
 // 0x40000000 - 0x5FFFFFFF: STM32 peripherals
 // 0xE0000000 - 0xFFFFFFFF: Cortex M7 internal peripherals
 MPU_InitStruct.SubRegionDisable = 0x84;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

 HAL_MPU_ConfigRegion(&MPU_InitStruct);

 /* Configure the MPU as cacheable non executable SRAM */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0x20000000;
 MPU_InitStruct.Size = MPU_REGION_SIZE_512MB;
 MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER1;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
 MPU_InitStruct.SubRegionDisable = 0;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

 HAL_MPU_ConfigRegion(&MPU_InitStruct);

 /* Configure the MPU as cacheable executable Flash */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0x08000000;
 MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
 MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER2;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
 MPU_InitStruct.SubRegionDisable = 0;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

 HAL_MPU_ConfigRegion(&MPU_InitStruct);

 /* Configure the MPU attributes options bytes */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0x1FF00000;
 MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
 MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER3;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
 MPU_InitStruct.SubRegionDisable = 0x0;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

 HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes for PSRAM as cacheable non executable RAM
   * !!! Write-through cannot be used as Cortex-M7 has an errata on it.
   * Write-through no allocate offer good PSRAM performance while not slowing
   * down audio processing too much, but has an errata on Cortex M7
   * So use no cache normal instead for minimal impact on audio.
   * mode            audio   timers
   * no cache ord    17     75
   * no cache normal 17     74
   * write-b no-a    19     27
   * write-b alloc   20     25
   * write-t no-a    17.5   34
   *
   * mode            usb    audio   timers  total
   * no cache ord    4.43   16      2.5     23
   * no cache normal 4.43   16      2.4     23
   * write-b no-a    4.44   16      0.92    21.7
   * write-b alloc
   * write-t no-a    4.44   16      1.2     22
   */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0x60000000;
 MPU_InitStruct.Size = MPU_REGION_SIZE_512KB; // PSRAM has 1MB but only 512KB is available due to wiring.
 MPU_InitStruct.AccessPermission = enable_psram ? MPU_REGION_FULL_ACCESS : MPU_REGION_NO_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER4;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
 MPU_InitStruct.SubRegionDisable = 0x00;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

 HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes for LCD Normal memory, Shareable */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0x64000000;
 MPU_InitStruct.Size = MPU_REGION_SIZE_32B;
 MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; // Don't cache LCD data
 MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER5;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
 MPU_InitStruct.SubRegionDisable = 0x00;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

 HAL_MPU_ConfigRegion(&MPU_InitStruct);

 /* Configure the MPU QSPI flash */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0x90000000;
 MPU_InitStruct.Size = MPU_REGION_SIZE_64MB;
 MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER6;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
 MPU_InitStruct.SubRegionDisable = 0x0;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;


 HAL_MPU_ConfigRegion(&MPU_InitStruct);

 /* Configure the MPU attributes FMC control registers */
 MPU_InitStruct.Enable = MPU_REGION_ENABLE;
 MPU_InitStruct.BaseAddress = 0xA0000000;
 MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;
 MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
 MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
 MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
 MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
 MPU_InitStruct.Number = MPU_REGION_NUMBER7;
 MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
 MPU_InitStruct.SubRegionDisable = 0x0;
 MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

 HAL_MPU_ConfigRegion(&MPU_InitStruct);

 /* Enable the MPU */
 HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

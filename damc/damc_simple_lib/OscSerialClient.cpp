#include "OscSerialClient.h"
#include <spdlog/spdlog.h>

#include <math.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include <stm32f723e_discovery.h>

static UART_HandleTypeDef uart;
static OscSerialClient* instance;

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
extern "C" void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&uart);
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	instance->onReceiveItCompleted(Size);
}

OscSerialClient::OscSerialClient(OscRoot* oscRoot)
    : OscConnector(oscRoot, true) {

	instance = this;

	SPDLOG_INFO("New Serial client");

	uart.Init.BaudRate = 115200;
	uart.Init.WordLength = UART_WORDLENGTH_8B;
	uart.Init.StopBits = UART_STOPBITS_1;
	uart.Init.Parity = UART_PARITY_NONE;
	uart.Init.Mode = UART_MODE_TX_RX;
	uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart.Init.OverSampling = UART_OVERSAMPLING_16;
	uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	BSP_COM_Init(COM1, &uart);

}


void OscSerialClient::sendOscData(const uint8_t* data, size_t size) {
    //HAL_UART_Transmit_IT(&uart, data, size);
}

void OscSerialClient::onReceiveItCompleted(uint16_t size) {
	onOscDataReceived(rx_buffer.data(), size);
	HAL_UART_Receive_IT(&uart, (uint8_t*) rx_buffer.data(), (uint16_t) rx_buffer.size());
}

void OscSerialClient::init() {
	HAL_UARTEx_ReceiveToIdle_IT(&uart, (uint8_t*) tx_buffer.data(), (uint16_t) tx_buffer.size());
}
void OscSerialClient::stop() {}

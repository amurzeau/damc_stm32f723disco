#include "OscSerialClient.h"
#include <spdlog/spdlog.h>

#include <math.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include <stm32f723e_discovery.h>
#include "usbd_cdc_if.h"

static UART_HandleTypeDef uart;
static OscSerialClient* instance;

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
    USB_CDC_IF_TX_write(data, size);
}

void OscSerialClient::mainLoop() {
    size_t read_size = USB_CDC_IF_RX_read(rx_buffer.data(), rx_buffer.size());
    if(read_size > 0) {
      onOscDataReceived(rx_buffer.data(), read_size);
    }
}

void OscSerialClient::init() {
}

void OscSerialClient::stop() {}

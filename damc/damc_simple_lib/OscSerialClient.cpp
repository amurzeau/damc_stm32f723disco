#include "OscSerialClient.h"
#include "TimeMeasure.h"

#include "usbd_cdc_if.h"

OscSerialClient::OscSerialClient(OscRoot* oscRoot) : OscConnector(oscRoot, true) {
	uv_async_init(uv_default_loop(), &asyncOnRxData, OscSerialClient::onRxDataStatic);
	asyncOnRxData.data = this;
	USB_CDC_IF_set_rx_data_ready_callback(OscSerialClient::onRxDataReadyFromIT, this);
}

inline bool isInterrupt() {
	return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

void OscSerialClient::sendOscData(const uint8_t* data, size_t size) {
	if(isInterrupt()) {
		while(1)
			;
	}
	USB_CDC_IF_TX_write(data, size);
}

void OscSerialClient::onRxDataReadyFromIT(void* arg) {
	OscSerialClient* thisInstance = (OscSerialClient*) arg;
	uv_async_send(&thisInstance->asyncOnRxData);
}

void OscSerialClient::onRxDataStatic(uv_async_t* handle) {
	OscSerialClient* thisInstance = (OscSerialClient*) handle->data;
	thisInstance->onRxData();
}

void OscSerialClient::onRxData() {
	TimeMeasure::timeMeasure[TMI_OscInput].beginMeasure();

	size_t read_size = USB_CDC_IF_RX_read(rx_buffer.data(), rx_buffer.size());
	if(read_size > 0) {
		onOscDataReceived(rx_buffer.data(), read_size);
	}

	TimeMeasure::timeMeasure[TMI_OscInput].endMeasure();
}

void OscSerialClient::init() {}

void OscSerialClient::stop() {}

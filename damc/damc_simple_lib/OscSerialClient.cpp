#include "OscSerialClient.h"
#include "TimeMeasure.h"

#include "usbd_cdc_if.h"

OscSerialClient::OscSerialClient(OscRoot* oscRoot) : OscConnector(oscRoot, true) {}

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

bool OscSerialClient::mainLoop() {
	size_t read_size = USB_CDC_IF_RX_read(rx_buffer.data(), rx_buffer.size());
	if(read_size > 0) {
		TimeMeasure::timeMeasure[TMI_OscInput].beginMeasure();
		onOscDataReceived(rx_buffer.data(), read_size);
		TimeMeasure::timeMeasure[TMI_OscInput].endMeasure();
		return true;
	}

	return false;
}

void OscSerialClient::init() {}

void OscSerialClient::stop() {}

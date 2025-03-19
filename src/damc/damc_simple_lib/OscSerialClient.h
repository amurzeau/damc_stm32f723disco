#pragma once

#include <OscRoot.h>
#include <array>
#include <stdint.h>
#include <uv.h>
#include <vector>

class OscSerialClient : public OscConnector {
public:
	OscSerialClient(OscRoot* oscRoot);

	void init();
	void stop();
	void sendOscData(const uint8_t* buffer, size_t sizeToSend) override;

protected:
	static void onRxDataReadyFromIT(void* arg);
	static void onRxDataStatic(uv_async_t* handle);
	void onRxData();

private:
	std::array<uint8_t, 256> rx_buffer;
	uv_async_t asyncOnRxData;
};

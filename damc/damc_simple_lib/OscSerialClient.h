#pragma once

#include <OscRoot.h>
#include <array>
#include <stdint.h>
#include <vector>

class OscSerialClient : public OscConnector {
public:
	OscSerialClient(OscRoot* oscRoot);

	void init();
	void stop();
	void sendOscData(const uint8_t* buffer, size_t sizeToSend) override;
	bool mainLoop();

protected:
private:
	std::array<uint8_t, 256> rx_buffer;
};

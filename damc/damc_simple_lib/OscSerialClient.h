#pragma once

#include <OscRoot.h>
#include <stdint.h>
#include <vector>
#include <array>

class OscSerialClient : public OscConnector {
public:
	OscSerialClient(OscRoot* oscRoot);

    void init();
    void stop();
	void sendOscData(const uint8_t* buffer, size_t sizeToSend) override;
	void mainLoop();

protected:

private:
	std::array<uint8_t, 256> rx_buffer;
};

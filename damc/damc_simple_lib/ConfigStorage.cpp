#include "ConfigStorage.h"
#include <spdlog/spdlog.h>

#include <stm32f723e_discovery.h>
#include <stm32f723e_discovery_qspi.h>
#include <string.h>

ConfigStorage::ConfigStorage(OscRoot* oscRoot) : oscRoot(oscRoot) {}

void ConfigStorage::load() {
	BSP_QSPI_Init();
}

bool ConfigStorage::onSlowTimer() {
	return false;
}

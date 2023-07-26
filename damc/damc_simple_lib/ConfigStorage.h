#pragma once

#include <OscRoot.h>
#include <array>
#include <stdint.h>
#include <vector>

class ConfigStorage {
public:
	ConfigStorage(OscRoot* oscRoot);

	void load();

	bool onSlowTimer();

protected:
private:
	OscRoot* oscRoot;
};

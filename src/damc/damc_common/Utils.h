#pragma once

#include <string_view>
#include <vector>
#include <stdint.h>

namespace Utils {

int32_t stringviewToNumber(std::string_view s);
bool isNumber(std::string_view s);

std::string_view toString(uint32_t value);

#define SPDLOG_LOG_WITH_LEVEL(level, ...) SPDLOG_LOGGER_CALL(spdlog::default_logger_raw(), level, __VA_ARGS__)

template<typename T> bool vector_find(const std::vector<T>& keys, const T& key) {
	for(const auto& existingKey : keys) {
		if(existingKey == key) {
			return true;
		}
	}
	return false;
}

template<typename T> void vector_erase(std::vector<T>& v, T value) {
	for(auto it = v.begin(); it != v.end();) {
		if(*it == value) {
			it = v.erase(it);
		} else {
			++it;
		}
	}
}

}  // namespace Utils

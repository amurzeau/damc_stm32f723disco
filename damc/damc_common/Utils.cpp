#include "Utils.h"
#include <charconv>
#include <memory>
#include <string>

int32_t Utils::stringviewToNumber(std::string_view s) {
	int32_t number;
	auto result = std::from_chars(s.begin(), s.end(), number);
	if(result.ec != std::errc{} || result.ptr != s.end()) {
		return 0;
	}
	return number;
}

bool Utils::isNumber(std::string_view s) {
	for(char c : s) {
		if(c < '0' || c > '9')
			return false;
	}
	// empty string not considered as a number
	return !s.empty();
}

std::string_view Utils::toString(uint32_t value) {
	static std::vector<std::unique_ptr<std::string>> existing_strings;
	if(value >= existing_strings.size()) {
		existing_strings.resize(value + 1);
	}

	std::unique_ptr<std::string>& valueStr = existing_strings[value];
	if(!valueStr) {
		valueStr.reset(new std::string(std::to_string(value)));
	}

	return *valueStr;
}

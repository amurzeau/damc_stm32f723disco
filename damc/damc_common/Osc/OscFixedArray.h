#pragma once

#include "OscContainer.h"
#include "OscFlatArray.h"
#include <array>
#include <initializer_list>

template<typename T, size_t N> class OscFixedArray : public OscContainer {
public:
	OscFixedArray(OscContainer* parent, std::string_view name, std::array<T, N>* value) noexcept;

	T& at(size_t index) { return value->at(index); }
	const T& at(size_t index) const { return value->at(index); }
	bool contains(size_t index) const;
	bool containsStr(std::string_view indexStr) const;

	auto size() const { return value->size(); }
	auto begin() const { return value->begin(); }
	auto begin() { return value->begin(); }
	auto end() const { return value->end(); }
	auto end() { return value->end(); }

private:
	OscFlatArray<int32_t> keys;
	std::array<T, N>* value;
};

template<typename T, size_t N>
OscFixedArray<T, N>::OscFixedArray(OscContainer* parent, std::string_view name, std::array<T, N>* value) noexcept
    : OscContainer(parent, name), keys(this, "keys"), value(value) {
	std::vector<int32_t> keysValue;

	for(size_t i = 0; i < N; i++) {
		keysValue.push_back(i);
	}
	keys.setData(std::move(keysValue));

	keys.addChangeCallback(
	    [this](const std::vector<int32_t>& oldKeys, const std::vector<int32_t>& newKeys) { return false; });
}

template<typename T, size_t N> bool OscFixedArray<T, N>::contains(size_t index) const {
	std::string_view indexStr = Utils::toString(index);
	return containsStr(indexStr);
}

template<typename T, size_t N> bool OscFixedArray<T, N>::containsStr(std::string_view indexStr) const {
	for(const auto& item : *value) {
		if(item->getName() == indexStr) {
			return true;
		}
	}
	return false;
}

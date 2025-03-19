#pragma once

#include "OscContainer.h"
#include <functional>
#include <stdint.h>
#include <string>
#include <vector>

template<typename T> class OscReadOnlyVariable : public OscContainer {
public:
	using underlying_type = T;
	using readonly_type = typename OscReadOnlyType<T>::type;

	using OscContainer::getFullAddress;
	using OscContainer::getName;

	OscReadOnlyVariable(OscContainer* parent, std::string_view name, readonly_type initialValue = {});
	OscReadOnlyVariable(const OscReadOnlyVariable&) = delete;

	void set(readonly_type v, bool fromOsc = false);
	void setFromOsc(readonly_type value);
	void setDefault(readonly_type v);
	void forceDefault(readonly_type v);

	T& get() { return value; }
	const T& get() const { return value; }
	readonly_type getToOsc() const;
	bool isDefault() const { return isDefaultValue; }

	template<typename U = T> std::enable_if_t<std::is_same_v<U, std::string>, const char*> c_str();

	operator T() const { return value; }
	void dump() override { notifyOsc(); }

	using OscContainer::operator=;
	OscReadOnlyVariable& operator=(readonly_type v);
	OscReadOnlyVariable& operator=(const OscReadOnlyVariable<T>& v);

	template<typename U = T>
	std::enable_if_t<std::is_same_v<U, std::string>, OscReadOnlyVariable<T>&> operator=(const char* v);
	bool operator==(const OscReadOnlyVariable<T>& other) { return value == other.value; }
	bool operator!=(const OscReadOnlyVariable<T>& other) { return !(*this == other); }

	void setOscConverters(std::function<readonly_type(readonly_type)> convertToOsc,
	                      std::function<readonly_type(readonly_type)> convertFromOsc);

	void addCheckCallback(std::function<bool(readonly_type)> onChange);
	void addChangeCallback(std::function<void(readonly_type)> onChange);

	void callChangeCallbacks(readonly_type v);
	bool callCheckCallbacks(readonly_type v);

protected:
	void notifyOsc();

private:
	T value{};

	std::function<readonly_type(readonly_type)> convertToOsc;
	std::function<readonly_type(readonly_type)> convertFromOsc;
	std::vector<std::function<bool(readonly_type)>> checkCallbacks;
	std::vector<std::function<void(readonly_type)>> onChangeCallbacks;
	bool isDefaultValue;
};

EXPLICIT_INSTANCIATE_OSC_VARIABLE(extern template, OscReadOnlyVariable)

template<typename T>
template<typename U>
std::enable_if_t<std::is_same_v<U, std::string>, OscReadOnlyVariable<T>&> OscReadOnlyVariable<T>::operator=(
    const char* v) {
	set(v);
	return *this;
}

template<typename T>
template<typename U>
std::enable_if_t<std::is_same_v<U, std::string>, const char*> OscReadOnlyVariable<T>::c_str() {
	return value.c_str();
}

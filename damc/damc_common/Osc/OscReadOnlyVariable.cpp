#include "OscReadOnlyVariable.h"
#include "OscRoot.h"
#include <spdlog/spdlog.h>

EXPLICIT_INSTANCIATE_OSC_VARIABLE(template, OscReadOnlyVariable)

template<typename T>
OscReadOnlyVariable<T>::OscReadOnlyVariable(OscContainer* parent, std::string_view name, readonly_type initialValue)
    : OscContainer(parent, name), value(initialValue), isDefaultValue(true) {
	if(getRoot()->isOscValueAuthority())
		notifyOsc();
}

template<typename T> void OscReadOnlyVariable<T>::set(readonly_type v, bool fromOsc) {
	if(value != v || isDefaultValue) {
		bool isDataValid = callCheckCallbacks(v);
		if(isDataValid) {
			SPDLOG_INFO("{}: set to {}", getFullAddress(), v);
			isDefaultValue = false;
			value = v;
			callChangeCallbacks(v);
			if(!fromOsc || getRoot()->isOscValueAuthority())
				notifyOsc();
		} else {
			SPDLOG_WARN("{}: refused invalid value {}", getFullAddress(), v);

			if(fromOsc) {
				// Ensure the client that set this is notified that the value didn't changed
				notifyOsc();
			}
		}
	}
}

template<typename T> void OscReadOnlyVariable<T>::setDefault(readonly_type v) {
	if(isDefaultValue) {
		isDefaultValue = false;  // Only notify if the value is different
		set(v);
		isDefaultValue = true;
	}
}

template<typename T> void OscReadOnlyVariable<T>::forceDefault(readonly_type v) {
	isDefaultValue = false;  // Only notify if the value is different
	set(v);
	isDefaultValue = true;
}

template<typename T> OscReadOnlyVariable<T>& OscReadOnlyVariable<T>::operator=(readonly_type v) {
	set(v);
	return *this;
}

template<typename T> OscReadOnlyVariable<T>& OscReadOnlyVariable<T>::operator=(const OscReadOnlyVariable<T>& v) {
	set(v.value);
	return *this;
}

template<typename T>
void OscReadOnlyVariable<T>::setOscConverters(std::function<readonly_type(readonly_type)> convertToOsc,
                                              std::function<readonly_type(readonly_type)> convertFromOsc) {
	this->convertToOsc = convertToOsc;
	this->convertFromOsc = convertFromOsc;
}

template<typename T> void OscReadOnlyVariable<T>::addCheckCallback(std::function<bool(readonly_type)> checkCallback) {
	this->checkCallbacks.push_back(checkCallback);
	checkCallback(this->get());
}

template<typename T> void OscReadOnlyVariable<T>::addChangeCallback(std::function<void(readonly_type)> onChange) {
	if(this->onChangeCallbacks.empty())
		this->onChangeCallbacks.reserve(2);
	this->onChangeCallbacks.push_back(onChange);
	onChange(this->get());
}

template<typename T> void OscReadOnlyVariable<T>::callChangeCallbacks(readonly_type v) {
	for(auto& callback : onChangeCallbacks) {
		callback(v);
	}
}

template<typename T> bool OscReadOnlyVariable<T>::callCheckCallbacks(readonly_type v) {
	bool isDataValid = true;
	for(auto& callback : checkCallbacks) {
		isDataValid = isDataValid && callback(v);
	}
	return isDataValid;
}

template<typename T> void OscReadOnlyVariable<T>::notifyOsc() {
	OscArgument valueToSend = getToOsc();
	sendMessage(&valueToSend, 1);
}

template<typename T> typename OscReadOnlyVariable<T>::readonly_type OscReadOnlyVariable<T>::getToOsc() const {
	if(!convertToOsc)
		return get();
	else
		return convertToOsc(get());
}

template<typename T> void OscReadOnlyVariable<T>::setFromOsc(readonly_type value) {
	if(!convertFromOsc)
		set(value, true);
	else
		set(convertFromOsc(value), true);
}

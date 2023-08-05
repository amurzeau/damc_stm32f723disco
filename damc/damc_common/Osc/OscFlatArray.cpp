#include "OscFlatArray.h"
#include "OscRoot.h"
#include <algorithm>
#include <spdlog/spdlog.h>
#include <type_traits>

EXPLICIT_INSTANCIATE_OSC_VARIABLE(template, OscFlatArray);

template<typename T>
OscFlatArray<T>::OscFlatArray(OscContainer* parent, std::string_view name) noexcept : OscContainer(parent, name) {
	this->getRoot()->addPendingConfigNode(this);
}

template<typename T> void OscFlatArray<T>::reserve(size_t reserveSize) {
	values.reserve(reserveSize);
}

template<typename T> const std::vector<T>& OscFlatArray<T>::getData() const {
	return values;
}

template<typename T> bool OscFlatArray<T>::setData(const std::vector<T>& newData) {
	return updateData([&newData](std::vector<T>& data) { data = newData; });
}

template<typename T> bool OscFlatArray<T>::setData(std::vector<T>&& newData) {
	return updateData([newData = std::move(newData)](std::vector<T>& data) { data = std::move(newData); });
}

template<typename T> std::string OscFlatArray<T>::getAsString() const {
	std::string result = "[";

	for(const auto& item : values) {
		if constexpr(std::is_same_v<T, std::string>) {
			result += " \"" + item + "\",";
		} else {
			result += " " + std::to_string(item) + ",";
		}
	}

	if(result.back() == ',')
		result.pop_back();

	return result + " ]";
}

template<typename T> void OscFlatArray<T>::execute(const std::vector<OscArgument>& arguments) {
	auto oldValues = values;

	updateData(
	    [this, &arguments](std::vector<T>& data) {
		    data.clear();
		    for(const auto& arg : arguments) {
			    T v;
			    if(this->template getArgumentAs<T>(arg, v)) {
				    data.push_back(v);
			    }
		    }
	    },
	    true);
}

template<typename T> void OscFlatArray<T>::addCheckCallback(std::function<bool(const std::vector<T>&)> checkCallback) {
	checkCallbacks.push_back(checkCallback);
	checkCallback(this->getData());
}

template<typename T>
void OscFlatArray<T>::addChangeCallback(std::function<void(const std::vector<T>&, const std::vector<T>&)> onChange) {
	if(this->onChangeCallbacks.empty())
		this->onChangeCallbacks.reserve(2);
	this->onChangeCallbacks.push_back(onChange);
	onChange({}, values);
}

template<typename T> bool OscFlatArray<T>::callCheckCallbacks(const std::vector<T>& v) {
	bool isDataValid = true;
	for(auto& callback : checkCallbacks) {
		isDataValid = isDataValid && callback(v);
	}
	return isDataValid;
}

template<typename T>
void OscFlatArray<T>::visit(const std::function<void(OscNode*, OscArgument*, size_t)>& nodeVisitorFunction) {
	size_t size = values.size();
	OscArgument valueToSend[size];

	for(size_t i = 0; i < size; i++) {
		valueToSend[i] = values[i];
	}
	nodeVisitorFunction(this, valueToSend, size);
}

template<typename T> void OscFlatArray<T>::notifyOsc() {
	// std::vector<OscArgument> valueToSend;
	size_t size = values.size();
	OscArgument valueToSend[size];

	for(size_t i = 0; i < size; i++) {
		valueToSend[i] = values[i];
	}
	sendMessage(&valueToSend[0], size);
}

template<typename T> bool OscFlatArray<T>::checkData(const std::vector<T>& savedValues, bool fromOsc) {
	if(values != savedValues) {
		bool isDataValid = callCheckCallbacks(values);
		if(isDataValid) {
			for(auto& callback : onChangeCallbacks) {
				callback(savedValues, values);
			}

			if(!fromOsc || getRoot()->isOscValueAuthority())
				notifyOsc();
			getRoot()->notifyValueChanged();

			return true;
		} else {
			SPDLOG_WARN("{}: refused invalid value", getFullAddress());

			values = savedValues;

			if(fromOsc) {
				// Ensure the client that set this is notified that the value didn't changed
				notifyOsc();
			}
		}
	}

	return false;
}

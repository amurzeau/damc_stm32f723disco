#include "OscDynamicFlatArray.h"
#include <spdlog/spdlog.h>

EXPLICIT_INSTANCIATE_OSC_VARIABLE(template, OscDynamicFlatArray)

template<typename T>
OscDynamicFlatArray<T>::OscDynamicFlatArray(OscContainer* parent, std::string_view name) : OscContainer(parent, name) {}

template<typename T> void OscDynamicFlatArray<T>::execute(const std::vector<OscArgument>& arguments) {
	if(onWriteCallback) {
		std::vector<readonly_type> values;
		SPDLOG_TRACE("{} = ", getFullAddress());

		values.reserve(values.size());
		for(const auto& v : arguments) {
			readonly_type value;
			if(!this->template getArgumentAs<readonly_type>(v, value)) {
				return;
			}
			SPDLOG_TRACE(" - {} ", value);
			values.push_back(value);
		}
		onWriteCallback(values);
	}
}

template<typename T> std::string OscDynamicFlatArray<T>::getAsString() const {
	return {};
}

template<typename T>
void OscDynamicFlatArray<T>::setReadCallback(std::function<std::vector<readonly_type>()> onReadCallback) {
	this->onReadCallback = onReadCallback;
}

template<typename T>
void OscDynamicFlatArray<T>::setWriteCallback(std::function<void(const std::vector<readonly_type>&)> onWriteCallback) {
	this->onWriteCallback = onWriteCallback;
}

template<typename T> void OscDynamicFlatArray<T>::notifyOsc() {
	if(!onReadCallback)
		return;

	std::vector<OscArgument> valueToSend;
	auto values = onReadCallback();

	SPDLOG_TRACE("{} = ", getFullAddress());

	valueToSend.reserve(values.size());
	for(const auto& v : values) {
		SPDLOG_TRACE(" - {} ", v);
		valueToSend.push_back(v);
	}

	sendMessage(&valueToSend[0], valueToSend.size());
}

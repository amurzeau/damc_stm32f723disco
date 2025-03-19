#include "OscDynamicVariable.h"
#include <spdlog/spdlog.h>

EXPLICIT_INSTANCIATE_OSC_VARIABLE(template, OscDynamicVariable)

template<typename T>
OscDynamicVariable<T>::OscDynamicVariable(OscContainer* parent, std::string_view name) : OscContainer(parent, name) {}

template<typename T> void OscDynamicVariable<T>::execute(const std::vector<OscArgument>& arguments) {
	if(onWriteCallback && !arguments.empty()) {
		readonly_type v;
		if(this->template getArgumentAs<readonly_type>(arguments[0], v)) {
			onWriteCallback(std::move(v));
		}
	}
}

template<typename T> std::string OscDynamicVariable<T>::getAsString() const {
	return {};
}

template<typename T> void OscDynamicVariable<T>::setReadCallback(std::function<readonly_type()> onReadCallback) {
	this->onReadCallback = onReadCallback;
}

template<typename T> void OscDynamicVariable<T>::setWriteCallback(std::function<void(readonly_type)> onWriteCallback) {
	this->onWriteCallback = onWriteCallback;
}

template<typename T> void OscDynamicVariable<T>::notifyOsc() {
	if(!onReadCallback)
		return;

	OscArgument valueToSend = onReadCallback();
	sendMessage(&valueToSend, 1);
}

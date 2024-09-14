#include "OscArray.h"
#include "Utils.h"

EXPLICIT_INSTANCIATE_OSC_VARIABLE(template, OscArray)

template<typename T>
OscArray<T>::OscArray(OscContainer* parent, std::string_view name, readonly_type defaultValue)
    : OscGenericArray<OscVariable<T>>(parent, name) {
	this->setFactory([defaultValue](OscContainer* parent, int name) {
		return new OscVariable<T>(parent, Utils::toString(name), defaultValue);
	});
}

template<typename T>
void OscArray<T>::setOscConverters(std::function<readonly_type(readonly_type)> convertToOsc,
                                   std::function<readonly_type(readonly_type)> convertFromOsc) {
	this->convertToOsc = convertToOsc;
	this->convertFromOsc = convertFromOsc;
}

template<typename T> void OscArray<T>::addChangeCallback(std::function<void(readonly_type)> onChangeCallbacks) {
	this->onChangeCallbacks.push_back(onChangeCallbacks);
}

template<typename T> void OscArray<T>::initializeItem(OscVariable<T>* item) {
	if(convertToOsc || convertFromOsc) {
		item->setOscConverters(convertToOsc, convertFromOsc);
	}
	for(auto& callback : onChangeCallbacks) {
		item->addChangeCallback(callback);
	}
}

#pragma once

#include "OscGenericArray.h"
#include "OscVariable.h"
#include <functional>

template<typename T> class OscArray : public OscGenericArray<OscVariable<T>> {
public:
	using readonly_type = typename OscVariable<T>::readonly_type;

	OscArray(OscContainer* parent, std::string_view name, readonly_type defaultValue = {});

	void setOscConverters(std::function<readonly_type(readonly_type)> convertToOsc,
	                      std::function<readonly_type(readonly_type)> convertFromOsc);
	void addChangeCallback(std::function<void(readonly_type)> onChangeCallbacks);

protected:
	void initializeItem(OscVariable<T>* item) override;

private:
	std::function<readonly_type(readonly_type)> convertToOsc;
	std::function<readonly_type(readonly_type)> convertFromOsc;
	std::vector<std::function<void(readonly_type)>> onChangeCallbacks;
};

EXPLICIT_INSTANCIATE_OSC_VARIABLE(extern template, OscArray)

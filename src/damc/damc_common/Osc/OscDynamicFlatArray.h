#pragma once

#include "OscContainer.h"
#include "OscReadOnlyVariable.h"

template<typename T> class OscDynamicFlatArray : public OscContainer {
public:
	using readonly_type = typename OscReadOnlyType<T>::type;

	OscDynamicFlatArray(OscContainer* parent, std::string_view name);
	OscDynamicFlatArray(const OscDynamicFlatArray&) = delete;

	void dump() override { notifyOsc(); }
	void execute(const std::vector<OscArgument>& arguments) override;

	std::string getAsString() const override;

	void setReadCallback(std::function<std::vector<readonly_type>()> onReadCallback);
	void setWriteCallback(std::function<void(const std::vector<readonly_type>&)> onWriteCallback);

protected:
	void notifyOsc();

private:
	std::function<std::vector<readonly_type>()> onReadCallback;
	std::function<void(const std::vector<readonly_type>&)> onWriteCallback;
};

EXPLICIT_INSTANCIATE_OSC_VARIABLE(extern template, OscDynamicFlatArray)

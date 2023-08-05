#pragma once

#include "OscContainer.h"
#include "OscReadOnlyVariable.h"

template<typename T> class OscDynamicVariable : public OscContainer {
public:
	using readonly_type = typename OscReadOnlyType<T>::type;

	OscDynamicVariable(OscContainer* parent, std::string_view name);
	OscDynamicVariable(const OscDynamicVariable&) = delete;

	void dump() override { notifyOsc(); }
	void execute(const std::vector<OscArgument>& arguments) override;

	std::string getAsString() const override;

	void setReadCallback(std::function<readonly_type()> onReadCallback);
	void setWriteCallback(std::function<void(readonly_type)> onWriteCallback);

protected:
	void notifyOsc();

private:
	std::function<readonly_type()> onReadCallback;
	std::function<void(readonly_type)> onWriteCallback;
};

EXPLICIT_INSTANCIATE_OSC_VARIABLE(extern template, OscDynamicVariable)

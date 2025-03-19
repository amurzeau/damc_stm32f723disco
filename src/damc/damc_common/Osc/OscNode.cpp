#include "OscNode.h"
#include "OscContainer.h"
#include "OscRoot.h"
#include <spdlog/spdlog.h>
#include <string_view>

template bool OscNode::getArgumentAs<bool>(const OscArgument& argument, bool& v);
template bool OscNode::getArgumentAs<int32_t>(const OscArgument& argument, int32_t& v);
template bool OscNode::getArgumentAs<float>(const OscArgument& argument, float& v);
template bool OscNode::getArgumentAs<std::string_view>(const OscArgument& argument, std::string_view& v);

OscNode::OscNode(OscContainer* parent, std::string_view name) noexcept : name(name), parent(nullptr) {
	setOscParent(parent);
}

OscNode::~OscNode() {
	if(parent)
		parent->removeChild(this, name);
}

template<typename T> bool OscNode::getArgumentAs(const OscArgument& argument, T& v) {
	bool ret = false;
	std::visit(
	    [&v, &ret](auto&& arg) -> void {
		    using U = std::decay_t<decltype(arg)>;
		    if constexpr(std::is_same_v<U, T>) {
			    v = arg;
			    ret = true;
		    } else if constexpr(!std::is_same_v<U, std::string_view> && !std::is_same_v<T, std::string_view>) {
			    v = (T) arg;
			    ret = true;
		    }
	    },
	    argument);

	if(!ret) {
		SPDLOG_ERROR("{}: Bad argument type: {} is not a {}",
		             this->getFullAddress(),
		             OscRoot::getArgumentVectorAsString(&argument, 1),
		             osc_type_name<T>::name);
	}

	return ret;
}

void OscNode::setOscParent(OscContainer* parent) {
	if(parent == this->parent)
		return;

	if(this->parent) {
		this->parent->removeChild(this, name);
		this->parent = nullptr;
	}

	if(parent) {
		parent->addChild(name, this);
	}
	this->parent = parent;
}

size_t OscNode::constructFullName(std::string* outputString) const {
	using namespace std::literals;

	size_t fullNameSize = 0;
	if(parent) {
		fullNameSize += parent->constructFullName(outputString);
	}

	if(!name.empty()) {
		fullNameSize += 1 + name.size();
		if(outputString) {
			outputString->append("/"sv);
			outputString->append(name.begin(), name.end());
		}
	}

	return fullNameSize;
}

OscNode* OscNode::getNode(std::string_view address) {
	if(address.empty() || address == "/") {
		return this;
	}

	return nullptr;
}

void OscNode::getFullAddress(std::string* output) const {
	output->clear();
	output->reserve(constructFullName(nullptr));
	constructFullName(output);
}

void OscNode::visit(const std::function<void(OscNode*, OscArgument*, size_t)>& nodeVisitorFunction) {}

void OscNode::sendMessage(const OscArgument* arguments, size_t number) {
	getRoot()->sendMessage(this, arguments, number);
}

void OscNode::execute(std::string_view address, const std::vector<OscArgument>& arguments) {
	if(address.empty() || address == "/") {
		execute(arguments);
	}
}

OscRoot* OscNode::getRoot() {
	if(parent)
		return parent->getRoot();
	return nullptr;
}

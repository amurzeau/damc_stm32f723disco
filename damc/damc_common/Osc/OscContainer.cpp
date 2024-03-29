#include "OscContainer.h"
#include "Utils.h"
#include <OscRoot.h>
#include <spdlog/spdlog.h>

bool OscContainer::osc_node_comparator::operator()(const std::string_view& x, const std::string_view& y) const {
	bool isXnumber = Utils::isNumber(x);
	bool isYnumber = Utils::isNumber(y);

	// Always put non number nodes first and order numbers by their numerical value
	if(!isXnumber && isYnumber)
		return true;
	else if(isXnumber && !isYnumber)
		return false;
	else if(isXnumber && isYnumber)
		return Utils::stringviewToNumber(x) < Utils::stringviewToNumber(y);
	else {
		return x < y;
	}
}

OscContainer::OscContainer(OscContainer* parent, std::string_view name, size_t reserveSize) noexcept
    : OscNode(parent, name), children(reserveSize + 1), oscDump(this, "dump") {
	oscDump.setCallback([this](auto) { dump(); });
}

OscContainer::~OscContainer() {
	auto childrenToDetach = std::move(children);

	for(auto& child : childrenToDetach) {
		child->setOscParent(nullptr);
	}
}

void OscContainer::splitAddress(std::string_view address,
                                std::string_view* childAddress,
                                std::string_view* remainingAddress) {
	size_t nextSlash = address.find('/');
	if(childAddress)
		*childAddress = address.substr(0, nextSlash);

	if(remainingAddress && nextSlash != std::string_view::npos && nextSlash + 1 < address.size()) {
		*remainingAddress = address.substr(nextSlash + 1);
	}
}

void OscContainer::execute(std::string_view address, const std::vector<OscArgument>& arguments) {
	if(address.empty() || address == "/") {
		SPDLOG_TRACE("Executing address {}", getFullAddress());
		execute(arguments);
	} else {
		std::string_view childAddress;
		std::string_view remainingAddress;

		splitAddress(address, &childAddress, &remainingAddress);

		if(childAddress == "*") {
			// Wildcard
			for(auto& child : children) {
				child->execute(remainingAddress, arguments);
			}
		} else if(childAddress == "**") {
			// Double Wildcard: for each child, pass the remaining + the current address for recursive wildcard

			// Skip duplicate **
			while(remainingAddress.size() >= 3 && remainingAddress.substr(0, 3) == "**/")
				splitAddress(remainingAddress, nullptr, &remainingAddress);

			execute(remainingAddress, arguments);

			for(auto& child : children) {
				child->execute(address, arguments);
			}
		} else {
			for(auto& child : children) {
				if(child->getName() == childAddress) {
					child->execute(remainingAddress, arguments);
					return;
				}
			}

			SPDLOG_WARN("Address {} not found from {}", childAddressStr, getFullAddress());
		}
	}
}

OscNode* OscContainer::getNode(std::string_view address) {
	if(address.empty() || address == "/") {
		SPDLOG_TRACE("Executing address {}", getFullAddress());
		return this;
	} else {
		std::string_view childAddress;
		std::string_view remainingAddress;

		splitAddress(address, &childAddress, &remainingAddress);

		for(auto& child : children) {
			if(child->getName() == childAddress) {
				return child->getNode(remainingAddress);
			}
		}

		SPDLOG_WARN("Address {} not found from {}", childAddressStr, getFullAddress());
	}
	return nullptr;
}

void OscContainer::visit(const std::function<void(OscNode*, OscArgument*, size_t)>& nodeVisitorFunction) {
	for(auto& child : children) {
		child->visit(nodeVisitorFunction);
	}
}

std::string OscContainer::getAsString() const {
	std::string result;
	static size_t depth = 0;

	std::string indent;
	indent.resize(depth, '\t');

	if(!children.empty()) {
		result += "{";

		depth += 1;
		size_t processedItems = 0;
		for(const auto& child : children) {
			std::string childData = child->getAsString();

			if(childData.empty()) {
				continue;
			}

			if(processedItems > 0) {
				result += ",\n";
			} else {
				result += "\n";
			}
			result += indent + "\t\"" + std::string(child->getName()) + "\": " + childData;
			processedItems++;
		}
		depth -= 1;

		if(processedItems == 0)
			result.clear();
		else {
			result += "\n";
			result += indent + "}";
		}
	}

	return result;
}

void OscContainer::addChild(std::string_view name, OscNode* child) {
	children.push_back(child);
}

void OscContainer::removeChild(OscNode* node, std::string_view name) {
	OscRoot* root = getRoot();
	if(root)
		root->nodeRemoved(node);

	for(size_t i = 0; i < children.size(); i++) {
		if(children[i]->getName() == name) {
			children.erase(children.begin() + i);
			return;
		}
	}
}

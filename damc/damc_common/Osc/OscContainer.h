#pragma once

#include "OscEndpoint.h"
#include "OscNode.h"
#include <vector>

template<typename T> class PreallocatedVector : public std::vector<T> {
public:
	using std::vector<T>::vector;
	PreallocatedVector(size_t reserveSize) { this->reserve(reserveSize); }
};

class OscContainer : public OscNode {
public:
	struct osc_node_comparator {
		bool operator()(const std::string_view& x, const std::string_view& y) const;
	};

public:
	OscContainer(OscContainer* parent, std::string_view name, size_t reserveSize = 6) noexcept;
	~OscContainer() override;

	void addChild(std::string_view name, OscNode* child);
	void removeChild(OscNode* node, std::string_view name);

	void splitAddress(std::string_view address, std::string_view* childAddress, std::string_view* remainingAddress);

	using OscNode::execute;
	void execute(std::string_view address, const std::vector<OscArgument>& arguments) override;
	virtual OscNode* getNode(std::string_view address) override;
	void visit(const std::function<void(OscNode*, OscArgument*, size_t)>& nodeVisitorFunction) override;

	std::string getAsString() const override;

private:
	PreallocatedVector<OscNode*> children;
	OscEndpoint oscDump;
};

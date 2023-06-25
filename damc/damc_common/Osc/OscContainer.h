#pragma once

#include "OscEndpoint.h"
#include "OscNode.h"
#include <map>

class OscContainer : public OscNode {
public:
	struct osc_node_comparator {
		bool operator()(const std::string_view& x, const std::string_view& y) const;
	};

public:
	OscContainer(OscContainer* parent, std::string_view name) noexcept;
	~OscContainer() override;

	void addChild(std::string_view name, OscNode* child);
	void removeChild(OscNode* node, std::string_view name);

	std::map<std::string_view, OscNode*, osc_node_comparator>& getChildren() { return children; }
	const std::map<std::string_view, OscNode*, osc_node_comparator>& getChildren() const { return children; }
	void splitAddress(std::string_view address, std::string_view* childAddress, std::string_view* remainingAddress);

	using OscNode::execute;
	void execute(std::string_view address, const std::vector<OscArgument>& arguments) override;
	bool visit(const std::function<bool(OscNode*)>* nodeVisitorFunction) override;

	std::string getAsString() const override;

private:
	std::map<std::string_view, OscNode*, osc_node_comparator> children;
	OscEndpoint oscDump;
};

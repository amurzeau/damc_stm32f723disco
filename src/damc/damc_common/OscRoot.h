#pragma once

#include <Osc/OscContainer.h>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <stdint.h>
#include <string>
#include <variant>

struct tosc_message_const;

class OscConnector;
class OscNode;

class OscRoot : public OscContainer {
public:
	OscRoot(bool notifyAtInit);
	~OscRoot();

	void onOscPacketReceived(const uint8_t* data, size_t size);

	void printAllNodes();
	void triggerAddress(const std::string_view& address);

	void addConnector(OscConnector* connector);
	void removeConnector(OscConnector* connector);
	void addValueChangedCallback(std::function<void()> onOscValueChanged);

	// Called by nodes
	void serializeMessage(const std::function<void(std::string_view, uint8_t*, size_t)>& processResult,
	                      const OscNode* node,
	                      const OscArgument* arguments,
	                      size_t number);
	void sendMessage(const OscNode* node, const OscArgument* argument, size_t number);
	bool isOscValueAuthority();
	void notifyValueChanged();

	void addPendingConfigNode(OscNode* node);
	void nodeRemoved(OscNode* node);
	void loadNodeConfig(const std::map<std::string_view, std::vector<OscArgument>>& configValues);
	void saveNodeConfig(const std::function<void(OscNode*, OscArgument*, size_t)>& nodeVisitorFunction);

	static std::string getArgumentVectorAsString(const OscArgument* arguments, size_t number);

protected:
	void executeMessage(tosc_message_const* osc);
	OscRoot* getRoot() override;

private:
	std::set<OscConnector*> connectors;
	std::string nodeFullAddress;
	std::unique_ptr<uint8_t[]> oscOutputMessage;
	size_t oscOutputMaxSize;
	std::vector<std::function<void()>> onOscValueChanged;
	bool doNotifyOscAtInit;
	std::function<void(std::string_view nodeFullAddress, uint8_t* data, size_t size)> sendSerializedMessage;

	std::set<OscNode*> nodesPendingConfig;
};

class OscConnector {
public:
	OscConnector(OscRoot* oscRoot, bool useSlipProtocol);
	virtual ~OscConnector();

	void sendOscMessage(const uint8_t* data, size_t size);

protected:
	void onOscDataReceived(const uint8_t* data, size_t size);
	virtual void sendOscData(const uint8_t* data, size_t size) = 0;

	OscRoot* getOscRoot() { return oscRoot; }

private:
	OscRoot* oscRoot;
	bool useSlipProtocol;
	bool oscIsEscaping;
	std::vector<uint8_t> oscInputBuffer;
	std::vector<uint8_t> oscOutputBuffer;
	bool discardNextMessage;
};

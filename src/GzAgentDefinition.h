#pragma once

#include <mavlink_hub_sdk/agent/AgentDefinition.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace pendarlab::sdk::mavlink_hub
{
  class GzAgentDefinition : public AgentDefinition
  {
  public:
    virtual AgentConfigParseResult parseConfig(const std::unordered_map<std::string, std::string>& cfg) const override;
    virtual std::unique_ptr<Agent> create(const AgentConfig& cfg, std::unique_ptr<IManagerResourceRequester> requester) const override;
  };
} // namespace pendarlab::sdk::mavlink_hub

#pragma once

#include <mavlink_hub_sdk/agent/AgentDefinition.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace pendarlab::app::mavlink_hub::plugin
{
  using pendarlab::sdk::mavlink_hub::Agent;
  using pendarlab::sdk::mavlink_hub::AgentConfig;
  using pendarlab::sdk::mavlink_hub::AgentDefinition;
  using pendarlab::sdk::mavlink_hub::AgentConfigParseResult;
  using pendarlab::sdk::mavlink_hub::IManagerResourceRequester;

  /** Factory for GzAgent. Parses the plugin config and creates GzAgent instances. */
  class GzAgentDefinition : public AgentDefinition
  {
  public:
    /** Parses the string-keyed config map into a GzAgentConfig. @param cfg raw config entries.
        @return a parse result carrying the parsed config on success, or an error otherwise. */
    virtual AgentConfigParseResult parseConfig(const std::unordered_map<std::string, std::string>& cfg) const override;
    /** Creates a GzAgent from a parsed config. @param cfg the parsed config (must be a GzAgentConfig).
        @param requester handed to the agent for endpoint acquisition. @return the new agent, or
        nullptr if cfg is not a GzAgentConfig. */
    virtual std::unique_ptr<Agent> create(const AgentConfig& cfg, std::unique_ptr<IManagerResourceRequester> requester) const override;
  };
} // namespace pendarlab::app::mavlink_hub::plugin

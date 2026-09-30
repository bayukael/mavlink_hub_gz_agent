#pragma once

#include <mavlink_hub_sdk/agent/Agent.h>
#include <mavlink_hub_sdk/agent/AgentConfig.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include "GzAgentConfig.h"

#include <memory>

namespace pendarlab::sdk::mavlink_hub
{
  class GzAgent : public Agent
  {
  public:
    GzAgent(const GzAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester);
    virtual ~GzAgent();
    GzAgent(GzAgent&&) noexcept;
    GzAgent& operator=(GzAgent&&) noexcept;

  protected:
    virtual bool doStart() override;
    virtual bool doStop() override;
    virtual bool doConfigure(const AgentConfig& config) override;

  private:
    struct GzAgentImpl;
    std::unique_ptr<GzAgentImpl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

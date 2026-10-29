#pragma once

#include <mavlink_hub_sdk/agent/Agent.h>
#include <mavlink_hub_sdk/agent/AgentConfig.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include "GzAgentConfig.h"

#include <memory>
#include <string>

namespace pendarlab::sdk::mavlink_hub
{
  /** The Mavlink <-> Gazebo bridge agent. Subscribes to Gazebo sensor topics and publishes
      actuator commands, bridging between a Mavlink endpoint and the simulation. */
  class GzAgent : public Agent
  {
  public:
    /** Constructs the agent, storing the resource requester, subscribing to the Gazebo sensor
        topics, and advertising the actuator topic. @param config the agent configuration.
        @param requester used to obtain the Mavlink endpoint at start time. */
    GzAgent(const GzAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester);
    /// Destroys the agent, stopping all background activity.
    virtual ~GzAgent();
    /// Move constructor (the impl is a unique_ptr).
    GzAgent(GzAgent&&) noexcept;
    /// Move assignment (the impl is a unique_ptr).
    GzAgent& operator=(GzAgent&&) noexcept;

    /** Returns the agent's single-line frequency report: the receive rate of every Gazebo
        topic it subscribes to, the send rate of the Gazebo topic it publishes, and the
        receive/send rate of the HIL actuator-control / HIL sensor MAVLink messages it
        receives/sends. Overrides the SDK Agent report. @return the report string. */
    std::string report() const override;

  protected:
    /// Requests the Mavlink endpoint, wires and starts the sensor send-loop and actuator listener.
    virtual bool doStart() override;
    /// Stops the send-loop and unregisters the actuator listener.
    virtual bool doStop() override;
    /// No-op configuration hook; configuration is fixed at construction.
    virtual bool doConfigure(const AgentConfig& config) override;

  private:
    struct GzAgentImpl;
    std::unique_ptr<GzAgentImpl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

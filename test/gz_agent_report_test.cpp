#include "GzAgent.h"
#include "GzAgentConfig.h"

#include <gtest/gtest.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <memory>
#include <string>
#include <vector>

using pendarlab::sdk::mavlink_hub::GzAgent;
using pendarlab::sdk::mavlink_hub::GzAgentConfig;
using pendarlab::sdk::mavlink_hub::IManagerResourceRequester;
using pendarlab::sdk::mavlink_hub::IMavlinkEndpointUser;

namespace
{
  /// A requester that never provides an endpoint: the report test never starts the agent,
  /// so no endpoint is ever requested.
  class NullResourceRequester : public IManagerResourceRequester
  {
  public:
    std::unique_ptr<IMavlinkEndpointUser> requestMavlinkEndpoint(const std::string&) override
    {
      return nullptr;
    }
    std::vector<std::string> getMavlinkEndpointList() override { return {}; }
  };

  /// Builds a fully-generated configuration, as the definition parser would produce.
  GzAgentConfig makeConfig()
  {
    GzAgentConfig config;
    config.vehicle_name = "x500";
    config.world_name = "test_world";
    config.mavlink_endpoint_name = "endpoint";
    config.generateTopics();
    return config;
  }
} // namespace

TEST(GzAgentReport, OverridesTheSdkDefaultReport)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  // The SDK default indicates "no report"; the gz agent carries real content.
  EXPECT_NE(agent.report(), "agent does not implement a report");
}

TEST(GzAgentReport, ReportIsASingleLine)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  const std::string text = agent.report();
  EXPECT_EQ(text.find('\n'), std::string::npos);
  EXPECT_EQ(text.find('\r'), std::string::npos);
  EXPECT_FALSE(text.empty());
}

TEST(GzAgentReport, GroupsGazeboSeparatelyFromMavlink)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  const std::string text = agent.report();
  const auto gz_begin = text.find("gz:");
  const auto group_separator = text.find('|');
  const auto mav_begin = text.find("mav:");
  ASSERT_NE(gz_begin, std::string::npos);
  ASSERT_NE(group_separator, std::string::npos);
  ASSERT_NE(mav_begin, std::string::npos);
  EXPECT_LT(gz_begin, group_separator);
  EXPECT_LT(group_separator, mav_begin);
}

TEST(GzAgentReport, CarriesEveryGazeboAndMavlinkStream)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  const std::string text = agent.report();
  for (const char* label : {"imu", "baro", "mag", "pose", "navsat", "actuator"}) {
    EXPECT_NE(text.find(std::string(label) + ":"), std::string::npos) << label;
  }
  for (const char* label : {"HIL_ACTUATOR_CONTROLS", "HIL_SENSOR"}) {
    EXPECT_NE(text.find(label), std::string::npos) << label;
  }
}

TEST(GzAgentReport, ReportsZeroRateBeforeAnyTraffic)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  // No Gazebo or MAVLink traffic has flowed, so every stream reads zero.
  EXPECT_EQ(agent.report(),
            "gz:imu:rx=0.0Hz/baro:rx=0.0Hz/mag:rx=0.0Hz/pose:rx=0.0Hz/navsat:rx=0.0Hz/"
            "actuator:tx=0.0Hz|mav:HIL_ACTUATOR_CONTROLS:rx=0.0Hz/HIL_SENSOR:tx=0.0Hz");
}

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

TEST(GzAgentReport, ReportIsASingleLineWithoutLabels)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  const std::string text = agent.report();
  EXPECT_EQ(text.find('\n'), std::string::npos);
  EXPECT_EQ(text.find('\r'), std::string::npos);
  EXPECT_FALSE(text.empty());
  // The report carries only digits and separators — no inline labels or unit text.
  for (char c : text) {
    EXPECT_TRUE(c == '0' || c == '/' || c == '|');
  }
}

TEST(GzAgentReport, GroupsAreInTheTxGzRxGzTxMavRxMavOrder)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  const std::string text = agent.report();
  // Four groups separated by '|' in the order tx_gz | rx_gz | tx_mav | rx_mav; the rx_gz
  // group bundles five values separated by '/'. Positions follow the README layout.
  ASSERT_EQ(text.size(), size_t(15));
  EXPECT_EQ(text, "0"          // tx_gz (actuator send rate)
                  "|"          // group separator
                  "0/0/0/0/0"  // rx_gz (imu, baro, mag, pose, navsat receive rates)
                  "|"          // group separator
                  "0"          // tx_mav (HIL_SENSOR send rate)
                  "|"          // group separator
                  "0");        // rx_mav (HIL_ACTUATOR_CONTROLS receive rate)
}

TEST(GzAgentReport, ReportsZeroRateBeforeAnyTraffic)
{
  GzAgent agent(makeConfig(), std::make_unique<NullResourceRequester>());
  // No Gazebo or MAVLink traffic has flowed, so every group reads zero.
  EXPECT_EQ(agent.report(), "0|0/0/0/0/0|0|0");
}

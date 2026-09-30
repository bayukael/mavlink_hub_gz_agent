#include "GzAgentDefinition.h"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace pendarlab::sdk::mavlink_hub
{
  namespace
  {
    GzAgentDefinition def;
  } // namespace

  TEST(GzAgentDefinitionParseConfig, MissingRequiredFieldsSetsParsedToNull)
  {
    auto result = def.parseConfig({});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(GzAgentDefinitionParseConfig, MissingVehicleNameSetsParsedToNull)
  {
    auto result = def.parseConfig({{"world_name", "w"}});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(GzAgentDefinitionParseConfig, MissingWorldNameSetsParsedToNull)
  {
    auto result = def.parseConfig({{"vehicle_name", "v"}});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(GzAgentDefinitionParseConfig, MissingMavlinkEndpointNameSetsParsedToNull)
  {
    auto result = def.parseConfig({
        {"vehicle_name", "v"},
        {"world_name", "w"},
        {"use_topic_generator", "true"},
    });

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(GzAgentDefinitionParseConfig, ValidConfigParses)
  {
    auto result = def.parseConfig({
        {"vehicle_name", "v"},
        {"world_name", "w"},
        {"use_topic_generator", "true"},
        {"mavlink_endpoint_name", "ep"},
    });

    EXPECT_TRUE(result.ok());
    EXPECT_NE(result.parsed, nullptr);
  }
} // namespace pendarlab::sdk::mavlink_hub

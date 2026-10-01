#include "GzAgentConfig.h"
#include "GzAgentDefinition.h"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace pendarlab::sdk::mavlink_hub
{
  namespace
  {
    GzAgentDefinition def;

    std::unordered_map<std::string, std::string> validBaseConfig()
    {
      return {
          {"vehicle_name", "v"},
          {"world_name", "w"},
          {"use_topic_generator", "true"},
          {"mavlink_endpoint_name", "ep"},
      };
    }
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
    auto result = def.parseConfig(validBaseConfig());

    EXPECT_TRUE(result.ok());
    EXPECT_NE(result.parsed, nullptr);
  }

  TEST(GzAgentDefinitionParseConfig, SendFreqDefaultsTo250)
  {
    auto result = def.parseConfig(validBaseConfig());

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const GzAgentConfig*>(result.parsed.get());
    EXPECT_DOUBLE_EQ(config->send_freq, 250.0);
  }

  TEST(GzAgentDefinitionParseConfig, SendFreqParsedWhenProvided)
  {
    auto cfg = validBaseConfig();
    cfg["send_freq"] = "100.0";
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const GzAgentConfig*>(result.parsed.get());
    EXPECT_DOUBLE_EQ(config->send_freq, 100.0);
  }

  TEST(GzAgentDefinitionParseConfig, SendFreqNonPositiveSetsParsedToNull)
  {
    auto cfg = validBaseConfig();
    cfg["send_freq"] = "0";
    auto result = def.parseConfig(cfg);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(GzAgentDefinitionParseConfig, SendFreqInvalidNumberSetsParsedToNull)
  {
    auto cfg = validBaseConfig();
    cfg["send_freq"] = "not_a_number";
    auto result = def.parseConfig(cfg);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }
} // namespace pendarlab::sdk::mavlink_hub

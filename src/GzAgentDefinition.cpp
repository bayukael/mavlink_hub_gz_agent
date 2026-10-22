#include "GzAgentDefinition.h"

#include "GzAgent.h"
#include "GzAgentConfig.h"

#include <exception>
#include <memory>
#include <string>
#include <unordered_map>

namespace pendarlab::app::mavlink_hub::plugin
{
  /** Parses the raw config map into a GzAgentConfig, validating required fields and the
      optional send_freq / magnetometer unit / frame values. @param cfg the raw key/value
      config entries. @return a parse result with the parsed config on success, or populated
      error messages otherwise. */
  AgentConfigParseResult GzAgentDefinition::parseConfig(const std::unordered_map<std::string, std::string>& cfg) const
  {
    AgentConfigParseResult result;
    auto config = std::make_unique<GzAgentConfig>();
    bool hasError = false;

    /// Returns the value of a required key, recording an error and returning "" if missing.
    auto getRequired = [&cfg, &result, &hasError](const std::string& key) -> std::string {
      auto it = cfg.find(key);
      if (it == cfg.end()) {
        result.messages.push_back("[GzAgent]: required entry is missing: " + key);
        hasError = true;
        return "";
      }
      return it->second;
    };

    config->vehicle_name = getRequired("vehicle_name");
    config->world_name = getRequired("world_name");
    config->mavlink_endpoint_name = getRequired("mavlink_endpoint_name");

    auto genIt = cfg.find("use_topic_generator");
    config->use_topic_generator = genIt != cfg.end() && (genIt->second == "true" || genIt->second == "1");

    auto unitIt = cfg.find("gz_magnetometer_unit");
    if (unitIt != cfg.end()) {
      if (unitIt->second == "tesla" || unitIt->second == "gauss") {
        config->gz_magnetometer_unit = unitIt->second;
      } else {
        result.messages.push_back("[GzAgent]: gz_magnetometer_unit must be 'tesla' or 'gauss': " + unitIt->second);
        hasError = true;
      }
    }

    auto frameIt = cfg.find("gz_magnetometer_frame");
    if (frameIt != cfg.end()) {
      if (frameIt->second == "ned" || frameIt->second == "enu") {
        config->gz_magnetometer_frame = frameIt->second;
      } else {
        result.messages.push_back("[GzAgent]: gz_magnetometer_frame must be 'ned' or 'enu': " + frameIt->second);
        hasError = true;
      }
    }

    auto freqIt = cfg.find("send_freq");
    if (freqIt != cfg.end()) {
      try {
        const double freq = std::stod(freqIt->second);
        if (freq <= 0.0) {
          result.messages.push_back("[GzAgent]: send_freq must be a positive number: " + freqIt->second);
          hasError = true;
        } else {
          config->send_freq = freq;
        }
      } catch (const std::exception&) {
        result.messages.push_back("[GzAgent]: send_freq is not a valid number: " + freqIt->second);
        hasError = true;
      }
    }

    if (config->use_topic_generator) {
      config->generateTopics();
    }

    /// Returns a topic: the user-supplied value if present, otherwise the generated default,
    /// recording an error if the topic is required but absent.
    auto getTopic = [&cfg, &result, &hasError](const std::string& key, const std::string& generated, bool required) -> std::string {
      auto it = cfg.find(key);
      if (it != cfg.end()) {
        return it->second;
      }
      if (required) {
        result.messages.push_back("[GzAgent]: required entry is missing: " + key);
        hasError = true;
        return "";
      }
      return generated;
    };

    bool requireTopics = !config->use_topic_generator;
    config->imu_topic = getTopic("imu_topic", config->imu_topic, requireTopics);
    config->baro_topic = getTopic("baro_topic", config->baro_topic, requireTopics);
    config->mag_topic = getTopic("mag_topic", config->mag_topic, requireTopics);
    config->pose_topic = getTopic("pose_topic", config->pose_topic, requireTopics);
    config->navsat_topic = getTopic("navsat_topic", config->navsat_topic, requireTopics);
    config->actuator_topic = getTopic("actuator_topic", config->actuator_topic, requireTopics);

    if (!hasError) {
      result.parsed = std::move(config);
    }

    return result;
  }

  /// Creates a GzAgent from a parsed config. @param cfg the parsed config (must be a
  /// GzAgentConfig). @param requester handed to the agent. @return the new agent, or
  /// nullptr if cfg is not a GzAgentConfig.
  std::unique_ptr<Agent> GzAgentDefinition::create(const AgentConfig& cfg, std::unique_ptr<IManagerResourceRequester> requester) const
  {
    const auto* gz_config = dynamic_cast<const GzAgentConfig*>(&cfg);
    if (gz_config == nullptr) {
      return nullptr;
    }
    return std::make_unique<GzAgent>(*gz_config, std::move(requester));
  }

  /// extern "C" factory entry point, dlsym'ed by mavlink_hub's LibLoader. @return a pointer
  /// to the singleton GzAgentDefinition.
  extern "C" const GzAgentDefinition* getGzAgentDefinition()
  {
    static GzAgentDefinition gz_agent_def;
    return &gz_agent_def;
  }
} // namespace pendarlab::app::mavlink_hub::plugin

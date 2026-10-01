#pragma once

#include <mavlink_hub_sdk/agent/AgentConfig.h>

#include <string>

namespace pendarlab::sdk::mavlink_hub
{
  class GzAgentConfig : public AgentConfig
  {
  public:
    std::string vehicle_name;
    std::string world_name;
    std::string imu_topic;
    std::string baro_topic;
    std::string mag_topic;
    std::string pose_topic;
    std::string navsat_topic;
    std::string actuator_topic;
    std::string mavlink_endpoint_name;
    double send_freq = 250.0;
    bool use_topic_generator = false;
    std::string gz_magnetometer_unit = "tesla";
    std::string gz_magnetometer_frame = "ned";

    void generateTopics()
    {
      imu_topic = "/world/" + world_name + "/model/" + vehicle_name + "/link/base_link/sensor/imu_sensor/imu";
      baro_topic = "/world/" + world_name + "/model/" + vehicle_name + "/link/base_link/sensor/air_pressure_sensor/air_pressure";
      mag_topic = "/world/" + world_name + "/model/" + vehicle_name + "/link/base_link/sensor/magnetometer_sensor/magnetometer";
      pose_topic = "/model/" + vehicle_name + "/pose";
      navsat_topic = "/world/" + world_name + "/model/" + vehicle_name + "/link/base_link/sensor/navsat_sensor/navsat";
      actuator_topic = "/" + vehicle_name + "/command/raw_actuators";
    }
  };
} // namespace pendarlab::sdk::mavlink_hub

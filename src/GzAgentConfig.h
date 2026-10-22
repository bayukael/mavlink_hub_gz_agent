#pragma once

#include <mavlink_hub_sdk/agent/AgentConfig.h>

#include <string>

namespace pendarlab::app::mavlink_hub::plugin
{
  using pendarlab::sdk::mavlink_hub::AgentConfig;

  /** Configuration for the Gazebo bridge agent. Subclasses AgentConfig to add the
      fields needed to connect a Mavlink endpoint to a Gazebo simulation. */
  class GzAgentConfig : public AgentConfig
  {
  public:
    /// Name of the simulated vehicle model in the Gazebo world.
    std::string vehicle_name;
    /// Name of the Gazebo world the vehicle lives in (used for topic generation / SDF lookup).
    std::string world_name;
    /// Gazebo topic publishing the vehicle's IMU.
    std::string imu_topic;
    /// Gazebo topic publishing the barometric air pressure.
    std::string baro_topic;
    /// Gazebo topic publishing the magnetometer field.
    std::string mag_topic;
    /// Gazebo topic publishing the vehicle pose.
    std::string pose_topic;
    /// Gazebo topic publishing the GPS/NavSat data.
    std::string navsat_topic;
    /// Gazebo topic on which actuator commands are published (Double_V).
    std::string actuator_topic;
    /// Name of the Mavlink endpoint to connect to.
    std::string mavlink_endpoint_name;
    /// Sensor send-loop frequency in Hz. Defaults to 250.
    double send_freq = 250.0;
    /// If true, fills the topic fields from vehicle_name + world_name instead of requiring them.
    bool use_topic_generator = false;
    /// Unit of the gz magnetometer field: "tesla" (default) or "gauss".
    std::string gz_magnetometer_unit = "tesla";
    /// Frame of the gz magnetometer field: "ned" (default) or "enu".
    std::string gz_magnetometer_frame = "ned";

    /// Derives all topic fields from vehicle_name and world_name.
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
} // namespace pendarlab::app::mavlink_hub::plugin

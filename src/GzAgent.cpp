#include "GzAgent.h"

#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <mavlink_hub_sdk/mavlink_endpoint_user/IMavlinkEndpointUser.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace pendarlab::sdk::mavlink_hub
{
  struct GzAgent::GzAgentImpl {
    GzAgentConfig config;

    std::unique_ptr<IManagerResourceRequester> requester;
    std::unique_ptr<IMavlinkEndpointUser> endpoint_user;

    gz::transport::Node node;
    gz::transport::Node::Publisher gz_actuator_pub;

    std::string vehicle_name;

    std::mutex imu_data_mutex;
    gz::msgs::IMU imu_data;
    bool imu_data_updated = false;
    std::chrono::steady_clock::time_point last_imu_timestamp;

    std::mutex baro_data_mutex;
    gz::msgs::FluidPressure baro_data;
    bool baro_data_updated = false;
    std::chrono::steady_clock::time_point last_baro_timestamp;

    std::mutex mag_data_mutex;
    gz::msgs::Magnetometer mag_data;
    bool mag_data_updated = false;
    std::chrono::steady_clock::time_point last_mag_timestamp;

    std::mutex pose_data_mutex;
    gz::msgs::Pose pose_data;
    bool pose_data_updated = false;
    std::chrono::steady_clock::time_point last_pose_timestamp;

    std::mutex navsat_data_mutex;
    gz::msgs::NavSat navsat_data;
    bool navsat_data_updated = false;
    std::chrono::steady_clock::time_point last_navsat_timestamp;

    void imu_callback(const gz::msgs::IMU& _msg)
    {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      imu_data = _msg;
      imu_data_updated = true;
      last_imu_timestamp = std::chrono::steady_clock::now();
    }

    void baro_callback(const gz::msgs::FluidPressure& _msg)
    {
      std::lock_guard<std::mutex> lock(baro_data_mutex);
      baro_data = _msg;
      baro_data_updated = true;
      last_baro_timestamp = std::chrono::steady_clock::now();
    }

    void mag_callback(const gz::msgs::Magnetometer& _msg)
    {
      std::lock_guard<std::mutex> lock(mag_data_mutex);
      mag_data = _msg;
      mag_data_updated = true;
      last_mag_timestamp = std::chrono::steady_clock::now();
    }

    void pose_callback(const gz::msgs::Pose_V& _msg)
    {
      std::lock_guard<std::mutex> lock(pose_data_mutex);
      for (size_t i = 0; i < _msg.pose_size(); i++) {
        if (_msg.pose(i).name() == vehicle_name) {
          pose_data = _msg.pose(i);
          pose_data_updated = true;
          last_pose_timestamp = std::chrono::steady_clock::now();
        }
      }
    }

    void navsat_callback(const gz::msgs::NavSat& _msg)
    {
      std::lock_guard<std::mutex> lock(navsat_data_mutex);
      navsat_data = _msg;
      navsat_data_updated = true;
      last_navsat_timestamp = std::chrono::steady_clock::now();
    }
  };

  GzAgent::GzAgent(const GzAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester)
      : d(std::make_unique<GzAgentImpl>())
  {
    d->config = config;
    d->vehicle_name = config.vehicle_name;
    d->requester = std::move(requester);

    d->node.Subscribe(config.imu_topic, &GzAgentImpl::imu_callback, d.get());
    d->node.Subscribe(config.baro_topic, &GzAgentImpl::baro_callback, d.get());
    d->node.Subscribe(config.mag_topic, &GzAgentImpl::mag_callback, d.get());
    d->node.Subscribe(config.pose_topic, &GzAgentImpl::pose_callback, d.get());
    d->node.Subscribe(config.navsat_topic, &GzAgentImpl::navsat_callback, d.get());

    d->gz_actuator_pub = d->node.Advertise<gz::msgs::Double_V>(config.actuator_topic);
  }

  GzAgent::~GzAgent() = default;
  GzAgent::GzAgent(GzAgent&&) noexcept = default;
  GzAgent& GzAgent::operator=(GzAgent&&) noexcept = default;

  bool GzAgent::doStart()
  {
    if (d->endpoint_user == nullptr) {
      d->endpoint_user = d->requester->requestMavlinkEndpoint(d->config.mavlink_endpoint_name);
      if (d->endpoint_user == nullptr) {
        return false;
      }
    }
    return true;
  }

  bool GzAgent::doStop()
  {
    return true;
  }

  bool GzAgent::doConfigure(const AgentConfig& config)
  {
    return true;
  }
} // namespace pendarlab::sdk::mavlink_hub

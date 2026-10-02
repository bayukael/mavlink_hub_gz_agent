#include "GzAgent.h"

#include "HilActuatorConverter.h"
#include "PeriodicSender.h"
#include "GzSensorConverter.h"

#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <mavlink/common/mavlink.h>
#include <mavlink_endpoint/MavlinkEndpointToken.h>
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

    PeriodicSender periodic_sender;
    std::unique_ptr<GzSensorConverter> gz_sensor_converter;

    gz::transport::Node node;
    gz::transport::Node::Publisher gz_actuator_pub;

    std::unique_ptr<pendarlab::lib::comm::MavlinkEndpointToken> actuator_listener;

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

    /// Stores the latest IMU sample under the imu mutex and marks it updated.
    void imu_callback(const gz::msgs::IMU& _msg)
    {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      imu_data = _msg;
      imu_data_updated = true;
      last_imu_timestamp = std::chrono::steady_clock::now();
    }

    /// Stores the latest barometer sample under the baro mutex and marks it updated.
    void baro_callback(const gz::msgs::FluidPressure& _msg)
    {
      std::lock_guard<std::mutex> lock(baro_data_mutex);
      baro_data = _msg;
      baro_data_updated = true;
      last_baro_timestamp = std::chrono::steady_clock::now();
    }

    /// Stores the latest magnetometer sample under the mag mutex and marks it updated.
    void mag_callback(const gz::msgs::Magnetometer& _msg)
    {
      std::lock_guard<std::mutex> lock(mag_data_mutex);
      mag_data = _msg;
      mag_data_updated = true;
      last_mag_timestamp = std::chrono::steady_clock::now();
    }

    /// Stores the pose of this vehicle (matched by name) from a Pose_V under the pose mutex.
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

    /// Stores the latest NavSat/GPS sample under the navsat mutex and marks it updated.
    void navsat_callback(const gz::msgs::NavSat& _msg)
    {
      std::lock_guard<std::mutex> lock(navsat_data_mutex);
      navsat_data = _msg;
      navsat_data_updated = true;
      last_navsat_timestamp = std::chrono::steady_clock::now();
    }
  };

  /// Stores the config/requester, subscribes to the Gazebo sensor topics, advertises the
  /// actuator topic, and configures the periodic send-loop period.
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

    d->gz_sensor_converter = std::make_unique<GzSensorConverter>(config);

    d->periodic_sender.setPeriod(std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 / config.send_freq)));
  }

  GzAgent::~GzAgent() = default;
  GzAgent::GzAgent(GzAgent&&) noexcept = default;
  GzAgent& GzAgent::operator=(GzAgent&&) noexcept = default;

  /// Requests the Mavlink endpoint, wires and starts the sensor send-loop, and registers
  /// the HIL_ACTUATOR_CONTROLS listener. @return false if the endpoint is unavailable.
  bool GzAgent::doStart()
  {
    if (d->endpoint_user == nullptr) {
      d->endpoint_user = d->requester->requestMavlinkEndpoint(d->config.mavlink_endpoint_name);
      if (d->endpoint_user == nullptr) {
        return false;
      }
    }

    d->periodic_sender.setSendFn([this](const mavlink_message_t& msg) {
      return d->endpoint_user != nullptr && d->endpoint_user->writeMessage(msg).has_value();
    });
    d->periodic_sender.setCollectFn([this]() {
      SensorSnapshot snap;
      {
        std::lock_guard<std::mutex> lock(d->imu_data_mutex);
        snap.imu = &d->imu_data;
        snap.imu_updated = d->imu_data_updated;
        d->imu_data_updated = false;
      }
      {
        std::lock_guard<std::mutex> lock(d->baro_data_mutex);
        snap.baro = &d->baro_data;
        snap.baro_updated = d->baro_data_updated;
        d->baro_data_updated = false;
      }
      {
        std::lock_guard<std::mutex> lock(d->mag_data_mutex);
        snap.mag = &d->mag_data;
        snap.mag_updated = d->mag_data_updated;
        d->mag_data_updated = false;
      }
      {
        std::lock_guard<std::mutex> lock(d->pose_data_mutex);
        snap.pose = &d->pose_data;
        snap.pose_updated = d->pose_data_updated;
        d->pose_data_updated = false;
      }
      {
        std::lock_guard<std::mutex> lock(d->navsat_data_mutex);
        snap.navsat = &d->navsat_data;
        snap.navsat_updated = d->navsat_data_updated;
        d->navsat_data_updated = false;
      }
      return d->gz_sensor_converter->convert(snap);
    });
    d->periodic_sender.start();

    // The endpoint invokes the callback on its own thread, so no separate
    // receive thread is needed. Hold the token alive for the lifetime of the
    // bridge; releasing it in doStop() unregisters the callback.
    //
    // NOTE: gz Publish() copies the message but may block until the first
    // subscriber connects or until queue space is available. Publishing here on
    // the endpoint's callback thread could stall that thread. Acceptable for
    // now; revisit if the endpoint stalls.
    d->actuator_listener = d->endpoint_user->createListener(
        [this](const pendarlab::lib::comm::MavlinkEndpointPacket& packet) {
          gz::msgs::Double_V msg;
          // Ignores messages with msgid != HIL_ACTUATOR_CONTROLS (no publish).
          if (tryConvertHilActuatorControlsToDoubleV(packet.msg, msg)) {
            d->gz_actuator_pub.Publish(msg);
          }
        });
    return d->actuator_listener != nullptr;
  }

  /// Stops the send-loop and releases the actuator listener token, unregistering the callback.
  /// @return true.
  bool GzAgent::doStop()
  {
    // Releasing the token unregisters the listener, so no dangling callback
    // or use-after-free can occur on stop/destruction.
    d->actuator_listener.reset();
    d->periodic_sender.stop();
    return true;
  }

  /// No-op configuration hook; the config is fixed at construction. @return true.
  bool GzAgent::doConfigure(const AgentConfig& config)
  {
    return true;
  }
} // namespace pendarlab::sdk::mavlink_hub

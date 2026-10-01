#pragma once

#include "GzAgentConfig.h"

#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/pose.pb.h>
#include <mavlink/common/mavlink.h>

#include <memory>
#include <vector>

namespace pendarlab::sdk::mavlink_hub
{
  // A snapshot of the latest sensor data plus a flag per sensor indicating whether it
  // has been updated since the last conversion. The gz::msgs pointers may be null;
  // conversion of a given sensor only happens when both the pointer and its *_updated
  // flag are set. All pointers refer to objects owned by the caller and must remain
  // valid for the duration of the convert() call.
  struct SensorSnapshot
  {
    const gz::msgs::IMU* imu = nullptr;
    const gz::msgs::FluidPressure* baro = nullptr;
    const gz::msgs::Magnetometer* mag = nullptr;
    const gz::msgs::Pose* pose = nullptr;
    const gz::msgs::NavSat* navsat = nullptr;

    bool imu_updated = false;
    bool baro_updated = false;
    bool mag_updated = false;
    bool pose_updated = false;
    bool navsat_updated = false;
  };

  // Converts a SensorSnapshot into HIL_SENSOR / HIL_GPS mavlink messages. The conversion
  // is deterministic: outputs are pure functions of the input snapshot + config (no RNG,
  // no sensor noise). The class holds no mutexes and no transport; it is unit-testable
  // directly from fixed inputs.
  class GzSensorConverter
  {
  public:
    explicit GzSensorConverter(const GzAgentConfig& cfg);
    ~GzSensorConverter();

    // Returns the messages to send for this snapshot (empty vector if nothing new).
    std::vector<mavlink_message_t> convert(const SensorSnapshot& snap);

  private:
    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

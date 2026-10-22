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

namespace pendarlab::app::mavlink_hub::plugin
{
  /** A snapshot of the latest sensor data plus a flag per sensor indicating whether it
      has been updated since the last conversion. The gz::msgs pointers may be null;
      conversion of a given sensor only happens when both the pointer and its *_updated
      flag are set. All pointers refer to objects owned by the caller and must remain
      valid for the duration of the convert() call. */
  struct SensorSnapshot
  {
    /// Latest IMU sample, or null if unavailable.
    const gz::msgs::IMU* imu = nullptr;
    /// Latest barometric pressure sample, or null if unavailable.
    const gz::msgs::FluidPressure* baro = nullptr;
    /// Latest magnetometer sample, or null if unavailable.
    const gz::msgs::Magnetometer* mag = nullptr;
    /// Latest pose sample, or null if unavailable.
    const gz::msgs::Pose* pose = nullptr;
    /// Latest NavSat/GPS sample, or null if unavailable.
    const gz::msgs::NavSat* navsat = nullptr;

    /// Whether imu has been updated since the last conversion.
    bool imu_updated = false;
    /// Whether baro has been updated since the last conversion.
    bool baro_updated = false;
    /// Whether mag has been updated since the last conversion.
    bool mag_updated = false;
    /// Whether pose has been updated since the last conversion.
    bool pose_updated = false;
    /// Whether navsat has been updated since the last conversion.
    bool navsat_updated = false;
  };

  /** Converts a SensorSnapshot into HIL_SENSOR / HIL_GPS mavlink messages. The conversion
      is deterministic: outputs are pure functions of the input snapshot + config (no RNG,
      no sensor noise). The class holds no mutexes and no transport; it is unit-testable
      directly from fixed inputs. */
  class GzSensorConverter
  {
  public:
    /** Constructs the converter, capturing the magnetometer unit/frame from the config.
        @param cfg the agent config (used for gz_magnetometer_unit / gz_magnetometer_frame). */
    explicit GzSensorConverter(const GzAgentConfig& cfg);
    /// Destroys the converter.
    ~GzSensorConverter();

    /** Converts a snapshot into sendable mavlink messages. @param snap the latest sensor
        data and update flags. @return the messages to send for this snapshot (empty vector
        if nothing new). */
    std::vector<mavlink_message_t> convert(const SensorSnapshot& snap);

  private:
    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::app::mavlink_hub::plugin

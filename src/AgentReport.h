#pragma once

#include "FrequencyCounter.h"

#include <memory>
#include <string>

namespace pendarlab::sdk::mavlink_hub
{
  /** Aggregates the frequency content of the agent report: the receive rate of every
      Gazebo topic the agent subscribes to (imu, baro, mag, pose, navsat), the send rate
      of the Gazebo topic it publishes (actuator), the receive rate of the
      HIL_ACTUATOR_CONTROLS MAVLink messages, and the send rate of the HIL_SENSOR MAVLink
      messages. build() renders the content as a single line, with gazebo-related data
      grouped together and kept separate from mavlink-related data.

      Thread-safe: the note*() methods are called from Gazebo/MAVLink callback threads and
      build() from the report-requesting thread. */
  class AgentReport
  {
  public:
    /** Constructs an empty report (all streams at zero rate until events are noted).
        @param clock the time source for the rate measurement; defaults to
        std::chrono::steady_clock::now(). Injectable so tests can drive deterministic
        frequencies. */
    explicit AgentReport(
        FrequencyCounter::Clock clock = std::chrono::steady_clock::now);
    /// Destroys the report.
    ~AgentReport();
    /// Move constructor.
    AgentReport(AgentReport&&) noexcept;
    /// Move assignment.
    AgentReport& operator=(AgentReport&&) noexcept;

    /// Records one IMU sensor sample received on the subscribed imu topic.
    void noteImu();
    /// Records one barometric sample received on the subscribed baro topic.
    void noteBaro();
    /// Records one magnetometer sample received on the subscribed mag topic.
    void noteMag();
    /// Records one pose sample of the agent's vehicle received on the subscribed pose topic.
    void notePose();
    /// Records one NavSat/GPS sample received on the subscribed navsat topic.
    void noteNavsat();
    /// Records one actuator command published on the actuator topic.
    void noteActuatorPublish();
    /// Records one HIL_ACTUATOR_CONTROLS MAVLink message received from the endpoint.
    void noteHilActuatorControlsReceive();
    /// Records one HIL_SENSOR MAVLink message sent to the endpoint.
    void noteHilSensorSend();

    /** Renders the report as a single line: the gazebo data points first, then the
        mavlink data points, '/' separating data points within a group and '|' separating
        the two groups. Every data point is <label>:<rx|tx>=<rate>Hz with one decimal.
        Reading the report resets the measured windows, so a later build() reports the
        interval since this one. @return the single-line report string. */
    std::string build();

  private:
    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

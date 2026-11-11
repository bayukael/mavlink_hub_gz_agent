#pragma once

#include "FrequencyCounter.h"

#include <memory>
#include <string>

namespace pendarlab::sdk::mavlink_hub
{
  /** Aggregates the frequency content of the agent report: the send rate of the Gazebo
      topic the agent publishes (tx_gz), the receive rate of every Gazebo topic it
      subscribes to (rx_gz), the send rate of the HIL_SENSOR MAVLink messages (tx_mav), and
      the receive rate of the HIL_ACTUATOR_CONTROLS MAVLink messages (rx_mav). build()
      renders the content as a single line of integer numbers separated only by separators,
      in the four-group order tx_gz|rx_gz|tx_mav|rx_mav. The meaning of each position is
      documented in README.md, not encoded as inline labels.

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

    /** Renders the report as a single line of integer rates (Hz) separated by '/' within a
        group and by '|' between groups, in the order tx_gz|rx_gz|tx_mav|rx_mav. Rates are
        rounded to the nearest integer. A stream with no measurable traffic reads 0.
        @return the single-line report string. */
    std::string build();

  private:
    struct AgentReportImpl;
    std::unique_ptr<AgentReportImpl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

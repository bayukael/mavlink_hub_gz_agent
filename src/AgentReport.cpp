#include "AgentReport.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace pendarlab::sdk::mavlink_hub
{
  namespace
  {
    /// Rounds a Hz rate to the nearest integer. @param hz the measured rate. @return the
    /// nearest whole number of Hz.
    std::int64_t roundHz(double hz)
    {
      return static_cast<std::int64_t>(std::llround(hz));
    }

    /// Renders one stream's rate as its integer Hz value. @param hz the measured rate.
    /// @return the integer Hz string.
    std::string rateString(double hz)
    {
      return std::to_string(roundHz(hz));
    }
  } // namespace

  struct AgentReport::AgentReportImpl
  {
    /// One counter per reported stream. build() groups them as tx_gz | rx_gz | tx_mav |
    /// rx_mav; within rx_gz the order is imu, baro, mag, pose, navsat.
    explicit AgentReportImpl(FrequencyCounter::Clock clock)
        : imu(clock), baro(clock), mag(clock), pose(clock), navsat(clock),
          actuator(clock), hil_actuator_controls(clock), hil_sensor(clock)
    {
    }

    FrequencyCounter actuator;            // tx_gz: send rate of the published actuator topic
    FrequencyCounter imu;                 // rx_gz
    FrequencyCounter baro;                // rx_gz
    FrequencyCounter mag;                 // rx_gz
    FrequencyCounter pose;                // rx_gz
    FrequencyCounter navsat;              // rx_gz
    FrequencyCounter hil_sensor;          // tx_mav: send rate of HIL_SENSOR
    FrequencyCounter hil_actuator_controls; // rx_mav: receive rate of HIL_ACTUATOR_CONTROLS
  };

  /// Constructs an empty report over the given time source. @param clock the time source.
  AgentReport::AgentReport(FrequencyCounter::Clock clock)
      : d(std::make_unique<AgentReportImpl>(std::move(clock)))
  {
  }

  /// Destroys the report.
  AgentReport::~AgentReport() = default;

  AgentReport::AgentReport(AgentReport&&) noexcept = default;
  AgentReport& AgentReport::operator=(AgentReport&&) noexcept = default;

  void AgentReport::noteImu() { d->imu.note(); }
  void AgentReport::noteBaro() { d->baro.note(); }
  void AgentReport::noteMag() { d->mag.note(); }
  void AgentReport::notePose() { d->pose.note(); }
  void AgentReport::noteNavsat() { d->navsat.note(); }
  void AgentReport::noteActuatorPublish() { d->actuator.note(); }
  void AgentReport::noteHilActuatorControlsReceive() { d->hil_actuator_controls.note(); }
  void AgentReport::noteHilSensorSend() { d->hil_sensor.note(); }

  /// Renders the single-line report: tx_gz | rx_gz | tx_mav | rx_mav, integer Hz values.
  std::string AgentReport::build()
  {
    const std::string tx_gz = rateString(d->actuator.frequency());
    const std::string rx_gz =
        rateString(d->imu.frequency()) + "/" +
        rateString(d->baro.frequency()) + "/" +
        rateString(d->mag.frequency()) + "/" +
        rateString(d->pose.frequency()) + "/" +
        rateString(d->navsat.frequency());
    const std::string tx_mav = rateString(d->hil_sensor.frequency());
    const std::string rx_mav = rateString(d->hil_actuator_controls.frequency());
    return tx_gz + "|" + rx_gz + "|" + tx_mav + "|" + rx_mav;
  }
} // namespace pendarlab::sdk::mavlink_hub

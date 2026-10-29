#include "AgentReport.h"

#include <cstdio>
#include <utility>

namespace pendarlab::sdk::mavlink_hub
{
  namespace
  {
    /// Formats one report data point. @param label the stream label. @param direction
    /// "rx" for received / "tx" for sent. @param hz the measured rate.
    /// @return "<label>:<direction>=<rate>Hz" with one decimal.
    std::string dataPoint(const char* label, const char* direction, double hz)
    {
      char rate[32];
      std::snprintf(rate, sizeof(rate), "%.1f", hz);
      return std::string(label) + ":" + direction + "=" + rate + "Hz";
    }
  } // namespace

  struct AgentReport::Impl
  {
    /// The streams are laid out in report order: the gazebo receive topics, the gazebo
    /// send topic, then the mavlink receive and send streams. build() renders the gazebo
    /// group and the mavlink group separately, in this order.
    explicit Impl(FrequencyCounter::Clock clock)
        : imu(clock), baro(clock), mag(clock), pose(clock), navsat(clock),
          actuator(clock), hil_actuator_controls(clock), hil_sensor(clock)
    {
    }

    FrequencyCounter imu;
    FrequencyCounter baro;
    FrequencyCounter mag;
    FrequencyCounter pose;
    FrequencyCounter navsat;
    FrequencyCounter actuator;
    FrequencyCounter hil_actuator_controls;
    FrequencyCounter hil_sensor;
  };

  /// Constructs an empty report over the given time source. @param clock the time source.
  AgentReport::AgentReport(FrequencyCounter::Clock clock)
      : d(std::make_unique<Impl>(std::move(clock)))
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

  /// Renders the single-line report: gazebo data points first, then mavlink data points.
  std::string AgentReport::build()
  {
    const std::string gazebo =
        dataPoint("imu", "rx", d->imu.frequency()) + "/" +
        dataPoint("baro", "rx", d->baro.frequency()) + "/" +
        dataPoint("mag", "rx", d->mag.frequency()) + "/" +
        dataPoint("pose", "rx", d->pose.frequency()) + "/" +
        dataPoint("navsat", "rx", d->navsat.frequency()) + "/" +
        dataPoint("actuator", "tx", d->actuator.frequency());
    const std::string mavlink =
        dataPoint("HIL_ACTUATOR_CONTROLS", "rx", d->hil_actuator_controls.frequency()) + "/" +
        dataPoint("HIL_SENSOR", "tx", d->hil_sensor.frequency());
    return "gz:" + gazebo + "|mav:" + mavlink;
  }
} // namespace pendarlab::sdk::mavlink_hub

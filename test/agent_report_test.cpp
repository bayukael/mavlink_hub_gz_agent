#include "AgentReport.h"

#include <gtest/gtest.h>

#include <chrono>
#include <string>

using pendarlab::sdk::mavlink_hub::AgentReport;

namespace
{
  /// A controllable clock: tests advance() time manually instead of sleeping, so the
  /// reported rates are exact.
  class FakeClock
  {
  public:
    std::chrono::steady_clock::time_point now() const { return now_; }
    void advance(std::chrono::steady_clock::duration duration) { now_ += duration; }

  private:
    std::chrono::steady_clock::time_point now_ = std::chrono::steady_clock::now();
  };

  /// Records one event on the given report stream, then advances the clock. Used to build
  /// a deterministic set of rates whose values are easy to predict.
  void noteAndAdvance(std::function<void()> note, FakeClock& clock,
                      std::chrono::steady_clock::duration step)
  {
    note();
    clock.advance(step);
  }
} // namespace

TEST(AgentReport, EmptyReportRendersAZeroPerGroup)
{
  AgentReport report;
  EXPECT_EQ(report.build(), "0|0/0/0/0/0|0|0");
}

TEST(AgentReport, ReportIsASingleLineWithoutLabels)
{
  AgentReport report;
  const std::string text = report.build();
  EXPECT_EQ(text.find('\n'), std::string::npos);
  EXPECT_EQ(text.find('\r'), std::string::npos);
  // The report carries only digits and separators — no letters (no inline labels).
  for (char c : text) {
    EXPECT_TRUE(c == '0' || c == '/' || c == '|');
  }
}

TEST(AgentReport, RendersInTheTxGzRxGzTxMavRxMavGroupOrder)
{
  AgentReport report;
  const std::string text = report.build();
  // Groups are separated by '|' in the order tx_gz | rx_gz | tx_mav | rx_mav; the rx_gz
  // group bundles five values separated by '/'.
  ASSERT_EQ(text.size(), size_t(15));
  EXPECT_EQ(text, "0"          // tx_gz (actuator)
                  "|"          // group separator
                  "0/0/0/0/0"  // rx_gz (imu, baro, mag, pose, navsat)
                  "|"          // group separator
                  "0"          // tx_mav (HIL_SENSOR)
                  "|"          // group separator
                  "0");        // rx_mav (HIL_ACTUATOR_CONTROLS)
}

TEST(AgentReport, ReportsRoundedIntegerRatesPerStream)
{
  FakeClock clock;
  AgentReport report([&clock]() { return clock.now(); });

  // Actuator (tx_gz): 5 events 0.125 s apart -> the 5-ts window spans 0.5 s -> 10 Hz.
  for (int i = 0; i < 5; i++) {
    noteAndAdvance([&]() { report.noteActuatorPublish(); }, clock, std::chrono::milliseconds(125));
  }
  // IMU (rx_gz 1): 5 events 0.25 s apart -> the window spans 1 s -> 5 Hz.
  for (int i = 0; i < 5; i++) {
    noteAndAdvance([&]() { report.noteImu(); }, clock, std::chrono::milliseconds(250));
  }
  // Baro (rx_gz 2): 5 events 0.25 s apart -> 5 Hz.
  for (int i = 0; i < 5; i++) {
    noteAndAdvance([&]() { report.noteBaro(); }, clock, std::chrono::milliseconds(250));
  }
  // Mag, pose, navsat (rx_gz 3-5): no events -> 0 Hz.
  // HIL_SENSOR (tx_mav): 5 events 0.125 s apart -> 10 Hz.
  for (int i = 0; i < 5; i++) {
    noteAndAdvance([&]() { report.noteHilSensorSend(); }, clock, std::chrono::milliseconds(125));
  }
  // HIL_ACTUATOR_CONTROLS (rx_mav): 5 events 1 s apart -> the window spans 4 s ->
  // 5/4 = 1.25 Hz, rounded to 1.
  for (int i = 0; i < 5; i++) {
    noteAndAdvance([&]() { report.noteHilActuatorControlsReceive(); }, clock, std::chrono::seconds(1));
  }

  EXPECT_EQ(report.build(), "10|5/5/0/0/0|10|1");
}

TEST(AgentReport, RoundsFractionsToTheNearestHz)
{
  FakeClock clock;
  AgentReport report([&clock]() { return clock.now(); });
  // NavSat (rx_gz 5): 5 events 0.3 s apart -> the 5-ts window spans 1.2 s -> 5/1.2 ≈ 4.17
  // Hz, rounded to 4.
  for (int i = 0; i < 5; i++) {
    noteAndAdvance([&]() { report.noteNavsat(); }, clock, std::chrono::milliseconds(300));
  }
  EXPECT_EQ(report.build(), "0|0/0/0/0/4|0|0");
}

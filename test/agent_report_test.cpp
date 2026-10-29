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

  /// Labels of every stream the report must carry, in report order within their groups.
  const char* const kGzLabels[] = {"imu", "baro", "mag", "pose", "navsat", "actuator"};
  const char* const kMavLabels[] = {"HIL_ACTUATOR_CONTROLS", "HIL_SENSOR"};
} // namespace

TEST(AgentReport, EmptyReportCarriesEveryStreamAtZeroRate)
{
  AgentReport report;
  EXPECT_EQ(report.build(),
            "gz:imu:rx=0.0Hz/baro:rx=0.0Hz/mag:rx=0.0Hz/pose:rx=0.0Hz/navsat:rx=0.0Hz/"
            "actuator:tx=0.0Hz|mav:HIL_ACTUATOR_CONTROLS:rx=0.0Hz/HIL_SENSOR:tx=0.0Hz");
}

TEST(AgentReport, ReportIsASingleLine)
{
  AgentReport report;
  const std::string text = report.build();
  EXPECT_EQ(text.find('\n'), std::string::npos);
  EXPECT_EQ(text.find('\r'), std::string::npos);
  EXPECT_FALSE(text.empty());
}

TEST(AgentReport, GazeboGroupIsSeparateFromMavlinkGroup)
{
  AgentReport report;
  const std::string text = report.build();
  // The gazebo group opens the line, the mavlink group follows it, and the '|' separates
  // the two groups; gazebo data is never mixed with mavlink data.
  const auto gz_begin = text.find("gz:");
  const auto group_separator = text.find('|');
  const auto mav_begin = text.find("mav:");
  ASSERT_NE(gz_begin, std::string::npos);
  ASSERT_NE(group_separator, std::string::npos);
  ASSERT_NE(mav_begin, std::string::npos);
  EXPECT_LT(gz_begin, group_separator);
  EXPECT_LT(group_separator, mav_begin);
}

TEST(AgentReport, CarriesEveryGazeboAndMavlinkStream)
{
  AgentReport report;
  const std::string text = report.build();
  for (const char* label : kGzLabels) {
    EXPECT_NE(text.find(std::string(label) + ":"), std::string::npos) << label;
  }
  for (const char* label : kMavLabels) {
    EXPECT_NE(text.find(label), std::string::npos) << label;
  }
}

TEST(AgentReport, ReportsMeasuredRatesPerStream)
{
  FakeClock clock;
  AgentReport report([&clock]() { return clock.now(); });

  for (int i = 0; i < 10; i++) {
    report.noteImu();
  }
  report.noteNavsat();
  for (int i = 0; i < 5; i++) {
    report.noteActuatorPublish();
  }
  for (int i = 0; i < 2; i++) {
    report.noteHilActuatorControlsReceive();
  }
  for (int i = 0; i < 7; i++) {
    report.noteHilSensorSend();
  }
  clock.advance(std::chrono::seconds(1));

  EXPECT_EQ(report.build(),
            "gz:imu:rx=10.0Hz/baro:rx=0.0Hz/mag:rx=0.0Hz/pose:rx=0.0Hz/navsat:rx=1.0Hz/"
            "actuator:tx=5.0Hz|mav:HIL_ACTUATOR_CONTROLS:rx=2.0Hz/HIL_SENSOR:tx=7.0Hz");
}

TEST(AgentReport, FormatsFractionalRatesWithOneDecimal)
{
  FakeClock clock;
  AgentReport report([&clock]() { return clock.now(); });
  report.noteBaro();
  clock.advance(std::chrono::milliseconds(300)); // 1 event over 0.3 s = 3.33 Hz
  EXPECT_NE(report.build().find("baro:rx=3.3Hz"), std::string::npos);
}

TEST(AgentReport, BuildResetsTheWindows)
{
  FakeClock clock;
  AgentReport report([&clock]() { return clock.now(); });
  report.noteMag();
  clock.advance(std::chrono::seconds(1));
  EXPECT_NE(report.build().find("mag:rx=1.0Hz"), std::string::npos);
  // No events since the previous build: the fresh windows are empty.
  EXPECT_NE(report.build().find("mag:rx=0.0Hz"), std::string::npos);
}

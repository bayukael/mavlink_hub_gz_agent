#include "FrequencyCounter.h"

#include <gtest/gtest.h>

#include <chrono>

using pendarlab::sdk::mavlink_hub::FrequencyCounter;

namespace
{
  /// A controllable clock: tests advance() time manually instead of sleeping, so the
  /// measured rates are exact.
  class FakeClock
  {
  public:
    std::chrono::steady_clock::time_point now() const { return now_; }
    void advance(std::chrono::steady_clock::duration duration) { now_ += duration; }

  private:
    std::chrono::steady_clock::time_point now_ = std::chrono::steady_clock::now();
  };

  /// Builds a counter driven by the given fake clock. @param clock the fake clock.
  FrequencyCounter makeCounter(FakeClock& clock)
  {
    return FrequencyCounter([&clock]() { return clock.now(); });
  }
} // namespace

TEST(FrequencyCounter, ReportsZeroWithoutAnyEvent)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  EXPECT_DOUBLE_EQ(counter.frequency(), 0.0);
}

TEST(FrequencyCounter, MeasuresAverageRateOverTheWindow)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  for (int i = 0; i < 5; i++) {
    counter.note();
  }
  clock.advance(std::chrono::seconds(1));
  EXPECT_DOUBLE_EQ(counter.frequency(), 5.0);
}

TEST(FrequencyCounter, CountsMultipleEventsAtTheSameInstant)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  counter.note();
  clock.advance(std::chrono::milliseconds(250));
  counter.note();
  clock.advance(std::chrono::milliseconds(250));
  counter.note();
  clock.advance(std::chrono::milliseconds(500)); // window spans 1 s with 3 events
  EXPECT_DOUBLE_EQ(counter.frequency(), 3.0);
}

TEST(FrequencyCounter, ZeroLengthWindowReportsZeroWithoutLosingEvents)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  counter.note();
  counter.note();
  // Read at the same clock reading as the events: the rate is undefined, so zero is
  // reported and the window is kept for the next read.
  EXPECT_DOUBLE_EQ(counter.frequency(), 0.0);
  clock.advance(std::chrono::seconds(2));
  EXPECT_DOUBLE_EQ(counter.frequency(), 1.0); // 2 events over the 2 s window
}

TEST(FrequencyCounter, ResetsTheWindowOnRead)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  counter.note();
  clock.advance(std::chrono::seconds(1));
  EXPECT_DOUBLE_EQ(counter.frequency(), 1.0);
  // No events since the read: the fresh window is empty.
  EXPECT_DOUBLE_EQ(counter.frequency(), 0.0);
}

TEST(FrequencyCounter, MeasuresEachIntervalSeparately)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  counter.note();
  counter.note();
  counter.note();
  clock.advance(std::chrono::milliseconds(500));
  EXPECT_DOUBLE_EQ(counter.frequency(), 6.0); // 3 events over 0.5 s

  counter.note();
  counter.note();
  clock.advance(std::chrono::milliseconds(600));
  EXPECT_NEAR(counter.frequency(), 2.0 / 0.6, 1e-9); // 2 events over 0.6 s
}

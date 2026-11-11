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

  /// Records one event at the current clock time, then advances. @param counter the
  /// counter. @param clock the fake clock. @param step the time to advance after the event.
  void noteAndAdvance(FrequencyCounter& counter, FakeClock& clock,
                      std::chrono::steady_clock::duration step)
  {
    counter.note();
    clock.advance(step);
  }
} // namespace

TEST(FrequencyCounter, ReportsZeroWithoutAnyEvent)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  EXPECT_DOUBLE_EQ(counter.frequency(), 0.0);
}

TEST(FrequencyCounter, SingleEventReportsZero)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  counter.note();
  clock.advance(std::chrono::seconds(1));
  // A window holding only one timestamp has no span to measure, so the rate is zero.
  EXPECT_DOUBLE_EQ(counter.frequency(), 0.0);
}

TEST(FrequencyCounter, MeasuresRateFromTheSlidingWindow)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  // Five evenly-spaced events over one second: window holds all five, span = 1 s.
  noteAndAdvance(counter, clock, std::chrono::milliseconds(250));
  noteAndAdvance(counter, clock, std::chrono::milliseconds(250));
  noteAndAdvance(counter, clock, std::chrono::milliseconds(250));
  noteAndAdvance(counter, clock, std::chrono::milliseconds(250));
  noteAndAdvance(counter, clock, std::chrono::milliseconds(0));
  // 5 timestamps over a 1 s span -> 5 Hz.
  EXPECT_DOUBLE_EQ(counter.frequency(), 5.0);
}

TEST(FrequencyCounter, KeepsOnlyTheNewestFiveSamples)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  // Ten events every 100 ms: only the last five timestamps (spanning 400 ms) are kept.
  for (int i = 0; i < 10; i++) {
    noteAndAdvance(counter, clock, std::chrono::milliseconds(100));
  }
  // 5 timestamps over a 0.4 s span -> 12.5 Hz.
  EXPECT_DOUBLE_EQ(counter.frequency(), 12.5);
}

TEST(FrequencyCounter, RateScalesWithSpanBetweenTwoEvents)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  noteAndAdvance(counter, clock, std::chrono::milliseconds(500));
  noteAndAdvance(counter, clock, std::chrono::milliseconds(0));
  // 2 timestamps over a 0.5 s span -> 4 Hz.
  EXPECT_DOUBLE_EQ(counter.frequency(), 4.0);
}

TEST(FrequencyCounter, ZeroLengthWindowReportsZero)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  counter.note();
  counter.note();
  // Both events share the same instant: no measurable span, so zero is reported.
  EXPECT_DOUBLE_EQ(counter.frequency(), 0.0);
}

TEST(FrequencyCounter, ReadingDoesNotResetTheWindow)
{
  FakeClock clock;
  auto counter = makeCounter(clock);
  noteAndAdvance(counter, clock, std::chrono::milliseconds(500));
  noteAndAdvance(counter, clock, std::chrono::milliseconds(0));
  EXPECT_DOUBLE_EQ(counter.frequency(), 4.0);
  // The window is unchanged by the read; the same rate is reported again.
  EXPECT_DOUBLE_EQ(counter.frequency(), 4.0);
}

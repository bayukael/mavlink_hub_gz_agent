#include "FrequencyCounter.h"

#include <mutex>
#include <queue>

namespace pendarlab::sdk::mavlink_hub
{
  struct FrequencyCounter::FrequencyCounterImpl
  {
    Clock clock;
    std::mutex mtx;
    // Sliding window of the timestamps of the kMaxSamples most recent events, oldest first.
    std::queue<std::chrono::steady_clock::time_point> samples;
  };

  /// Constructs a counter over the given time source. @param clock the time source.
  FrequencyCounter::FrequencyCounter(Clock clock) : d(std::make_unique<FrequencyCounterImpl>())
  {
    d->clock = std::move(clock);
  }

  /// Destroys the counter.
  FrequencyCounter::~FrequencyCounter() = default;

  FrequencyCounter::FrequencyCounter(FrequencyCounter&&) noexcept = default;
  FrequencyCounter& FrequencyCounter::operator=(FrequencyCounter&&) noexcept = default;

  /// Records one event, pushing its timestamp and trimming the window to the newest
  /// kMaxSamples events.
  void FrequencyCounter::note()
  {
    std::lock_guard<std::mutex> lock(d->mtx);
    d->samples.push(d->clock());
    while (d->samples.size() > kMaxSamples) {
      d->samples.pop();
    }
  }

  /// Computes the rate over the sliding window: stored timestamps divided by the span
  /// between the newest and oldest, or 0.0 when there is no measurable span.
  double FrequencyCounter::frequency()
  {
    std::lock_guard<std::mutex> lock(d->mtx);
    if (d->samples.size() < 2) {
      return 0.0;
    }
    const auto span = d->samples.back() - d->samples.front();
    if (span <= std::chrono::steady_clock::duration::zero()) {
      // All stored events share the same instant: no measurable span, so the rate is
      // undefined; report zero instead of dividing by zero.
      return 0.0;
    }
    return static_cast<double>(d->samples.size()) /
           std::chrono::duration<double>(span).count();
  }
} // namespace pendarlab::sdk::mavlink_hub

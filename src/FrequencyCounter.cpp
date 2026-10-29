#include "FrequencyCounter.h"

#include <mutex>

namespace pendarlab::sdk::mavlink_hub
{
  struct FrequencyCounter::Impl
  {
    Clock clock;
    std::mutex mtx;
    bool window_open = false;
    std::chrono::steady_clock::time_point window_start{};
    std::uint64_t count = 0;
  };

  /// Constructs a counter over the given time source. @param clock the time source.
  FrequencyCounter::FrequencyCounter(Clock clock) : d(std::make_unique<Impl>())
  {
    d->clock = std::move(clock);
  }

  /// Destroys the counter.
  FrequencyCounter::~FrequencyCounter() = default;

  FrequencyCounter::FrequencyCounter(FrequencyCounter&&) noexcept = default;
  FrequencyCounter& FrequencyCounter::operator=(FrequencyCounter&&) noexcept = default;

  /// Records one event, opening the measurement window on the first event of a window.
  void FrequencyCounter::note()
  {
    std::lock_guard<std::mutex> lock(d->mtx);
    if (!d->window_open) {
      d->window_start = d->clock();
      d->window_open = true;
    }
    ++d->count;
  }

  /// Reads the average rate over the current window and resets the window.
  double FrequencyCounter::frequency()
  {
    std::lock_guard<std::mutex> lock(d->mtx);
    if (!d->window_open) {
      return 0.0;
    }
    const auto now = d->clock();
    const auto elapsed = now - d->window_start;
    if (elapsed <= std::chrono::steady_clock::duration::zero()) {
      // Zero-length window: the rate is undefined; report zero instead of dividing
      // by zero, and keep the window so its events are not lost to the next read.
      return 0.0;
    }
    const double hz = static_cast<double>(d->count) /
                      std::chrono::duration<double>(elapsed).count();
    d->window_open = false;
    d->count = 0;
    return hz;
  }
} // namespace pendarlab::sdk::mavlink_hub

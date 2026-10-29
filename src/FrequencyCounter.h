#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>

namespace pendarlab::sdk::mavlink_hub
{
  /** Measures a stream's event rate (events per second) for the agent report, using a
      sliding window of the last kMaxSamples event timestamps. The clock is injectable so
      unit tests can drive deterministic frequencies instead of waiting in real time.
      Thread-safe: note() may be called from any thread (e.g. a Gazebo or MAVLink callback)
      and frequency() from any other thread (e.g. a report request). Reading the frequency
      does not reset the window, so successive reports reflect the recent activity. */
  class FrequencyCounter
  {
  public:
    /// Maximum number of event timestamps kept in the sliding window.
    static constexpr std::size_t kMaxSamples = 5;

    /// Time source used to mark events. Defaults to std::chrono::steady_clock.
    using Clock = std::function<std::chrono::steady_clock::time_point()>;

    /** Constructs a counter. @param clock the time source; defaults to
        std::chrono::steady_clock::now(). */
    explicit FrequencyCounter(Clock clock = std::chrono::steady_clock::now);
    /// Destroys the counter.
    ~FrequencyCounter();
    /// Move constructor.
    FrequencyCounter(FrequencyCounter&&) noexcept;
    /// Move assignment.
    FrequencyCounter& operator=(FrequencyCounter&&) noexcept;
    // Non-copyable: the impl owns a mutex.
    FrequencyCounter(const FrequencyCounter&) = delete;
    FrequencyCounter& operator=(const FrequencyCounter&) = delete;

    /** Records one event on the stream (one received/sent message or one Gazebo
        subscription sample). Safe to call from any thread. Keeps only the kMaxSamples most
        recent events. */
    void note();

    /** @return the event rate in Hertz over the current sliding window: the number of
        stored timestamps divided by the time between the newest and oldest of them.
        Returns 0.0 when fewer than two timestamps are stored (no measurable span). */
    double frequency();

  private:
    struct FrequencyCounterImpl;
    std::unique_ptr<FrequencyCounterImpl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

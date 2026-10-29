#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

namespace pendarlab::sdk::mavlink_hub
{
  /** Measures a stream's event rate (events per second) for the agent report. The clock
      is injectable so unit tests can drive deterministic frequencies instead of waiting
      in real time. Thread-safe: note() may be called from any thread (e.g. a Gazebo or
      MAVLink callback) and frequency() from any other thread (e.g. a report request).

      The measured window opens at the first event after the previous read. frequency()
      returns the average rate over that window and then resets it, so a sequence of
      reads reports the rate since the previous read — a fresh report always reflects
      the most recent activity. */
  class FrequencyCounter
  {
  public:
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
        subscription sample). Safe to call from any thread. */
    void note();

    /** @return the event rate in Hertz (events / seconds) over the current window, then
        resets the window. Returns 0.0 when no event has been recorded since the previous
        read or when the window has zero length (e.g. events and the read share the same
        clock reading). */
    double frequency();

  private:
    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

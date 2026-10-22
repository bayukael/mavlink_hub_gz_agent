#pragma once

#include <mavlink/common/mavlink.h>

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace pendarlab::app::mavlink_hub::plugin
{
  /** A background thread that periodically collects sendable messages and dispatches
      them through a send function. The class is conversion-agnostic: it only invokes
      the injected collect/send callbacks. It owns the thread lifecycle and the timing. */
  class PeriodicSender
  {
  public:
    /** Collects the messages that are ready to send this tick. An empty vector means
        nothing is sendable. The loop sends every message in the returned vector. */
    using CollectFn = std::function<std::vector<mavlink_message_t>()>;
    /** Sends a single message. The return value is ignored (all collected messages are
        sent even if an individual send fails). */
    using SendFn = std::function<bool(const mavlink_message_t&)>;

    /// Constructs an idle sender (not running).
    PeriodicSender();
    /// Stops the background loop (if running) and destroys the sender.
    ~PeriodicSender();
    /// Move constructor.
    PeriodicSender(PeriodicSender&&) noexcept;
    /// Move assignment.
    PeriodicSender& operator=(PeriodicSender&&) noexcept;

    /** Set the collect callback. Must be set BEFORE start(). @param fn callback producing
        the messages to send each tick. */
    void setCollectFn(CollectFn fn);
    /** Set the send callback. Must be set BEFORE start(). @param fn callback dispatching a
        single message. */
    void setSendFn(SendFn fn);
    /** Set the loop period. Must be set BEFORE start(). @param period time between ticks. */
    void setPeriod(std::chrono::microseconds period);
    /// @return the current loop period.
    std::chrono::microseconds period() const;

    /** Starts the background loop. No-op if already running. @return true once running. */
    bool start();
    /** Stops the background loop (sets a stop flag and joins the thread). No-op if not running. */
    void stop();
    /// @return true if the background loop is currently running.
    bool isRunning() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::app::mavlink_hub::plugin

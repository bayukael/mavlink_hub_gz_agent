#pragma once

#include <mavlink/common/mavlink.h>

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace pendarlab::sdk::mavlink_hub
{
  // A background thread that periodically collects sendable messages and dispatches
  // them through a send function. The class is conversion-agnostic: it only invokes
  // the injected collect/send callbacks. It owns the thread lifecycle and the timing.
  class PeriodicSender
  {
  public:
    // Collects the messages that are ready to send this tick. An empty vector means
    // nothing is sendable. The loop sends every message in the returned vector.
    using CollectFn = std::function<std::vector<mavlink_message_t>()>;
    // Sends a single message. The return value is ignored (all collected messages are
    // sent even if an individual send fails).
    using SendFn = std::function<bool(const mavlink_message_t&)>;

    PeriodicSender();
    ~PeriodicSender();
    PeriodicSender(PeriodicSender&&) noexcept;
    PeriodicSender& operator=(PeriodicSender&&) noexcept;

    // Set the collect/send callbacks and the period. These must be set BEFORE start().
    void setCollectFn(CollectFn fn);
    void setSendFn(SendFn fn);
    void setPeriod(std::chrono::microseconds period);
    std::chrono::microseconds period() const;

    // Starts the background loop. No-op if already running.
    bool start();
    // Stops the background loop (sets a stop flag and joins the thread). No-op if not running.
    void stop();
    bool isRunning() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::sdk::mavlink_hub

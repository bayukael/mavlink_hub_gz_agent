#include "PeriodicSender.h"

#include <atomic>
#include <thread>

namespace pendarlab::sdk::mavlink_hub
{
  struct PeriodicSender::Impl
  {
    std::thread thread;
    std::atomic<bool> stop{true};
    CollectFn collect_fn;
    SendFn send_fn;
    std::chrono::microseconds period{std::chrono::microseconds(4000)}; // 250 Hz default

    /// The background loop: until stopped, collect and send all messages each period.
    void routine()
    {
      while (!stop.load()) {
        const auto next = std::chrono::steady_clock::now() + period;
        if (collect_fn) {
          for (const auto& msg : collect_fn()) {
            if (send_fn) {
              send_fn(msg);
            }
          }
        }
        std::this_thread::sleep_until(next);
      }
    }
  };

  /// Constructs an idle (not running) sender.
  PeriodicSender::PeriodicSender() : d(std::make_unique<Impl>()) {}

  /// Stops the background loop if running, then destroys the sender.
  PeriodicSender::~PeriodicSender()
  {
    stop();
  }

  PeriodicSender::PeriodicSender(PeriodicSender&&) noexcept = default;
  PeriodicSender& PeriodicSender::operator=(PeriodicSender&&) noexcept = default;

  /// Stores the collect callback. @param fn callback producing messages for each tick.
  void PeriodicSender::setCollectFn(CollectFn fn)
  {
    d->collect_fn = std::move(fn);
  }

  /// Stores the send callback. @param fn callback dispatching a single message.
  void PeriodicSender::setSendFn(SendFn fn)
  {
    d->send_fn = std::move(fn);
  }

  /// Sets the loop period. @param period time between ticks.
  void PeriodicSender::setPeriod(std::chrono::microseconds period)
  {
    d->period = period;
  }

  /// @return the current loop period.
  std::chrono::microseconds PeriodicSender::period() const
  {
    return d->period;
  }

  /// Starts the background loop. @return true once running.
  bool PeriodicSender::start()
  {
    if (isRunning()) {
      return true;
    }
    d->stop.store(false);
    d->thread = std::thread(&Impl::routine, d.get());
    return true;
  }

  /// Stops the background loop, joining the thread.
  void PeriodicSender::stop()
  {
    if (!isRunning()) {
      return;
    }
    d->stop.store(true);
    if (d->thread.joinable()) {
      d->thread.join();
    }
  }

  /// @return true if the background loop is currently running.
  bool PeriodicSender::isRunning() const
  {
    return d->thread.joinable() && !d->stop.load();
  }
} // namespace pendarlab::sdk::mavlink_hub

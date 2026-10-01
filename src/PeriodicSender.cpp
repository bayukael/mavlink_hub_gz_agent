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

  PeriodicSender::PeriodicSender() : d(std::make_unique<Impl>()) {}

  PeriodicSender::~PeriodicSender()
  {
    stop();
  }

  PeriodicSender::PeriodicSender(PeriodicSender&&) noexcept = default;
  PeriodicSender& PeriodicSender::operator=(PeriodicSender&&) noexcept = default;

  void PeriodicSender::setCollectFn(CollectFn fn)
  {
    d->collect_fn = std::move(fn);
  }

  void PeriodicSender::setSendFn(SendFn fn)
  {
    d->send_fn = std::move(fn);
  }

  void PeriodicSender::setPeriod(std::chrono::microseconds period)
  {
    d->period = period;
  }

  std::chrono::microseconds PeriodicSender::period() const
  {
    return d->period;
  }

  bool PeriodicSender::start()
  {
    if (isRunning()) {
      return true;
    }
    d->stop.store(false);
    d->thread = std::thread(&Impl::routine, d.get());
    return true;
  }

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

  bool PeriodicSender::isRunning() const
  {
    return d->thread.joinable() && !d->stop.load();
  }
} // namespace pendarlab::sdk::mavlink_hub

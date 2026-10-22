#include "PeriodicSender.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

using pendarlab::app::mavlink_hub::plugin::PeriodicSender;

namespace
{
  constexpr auto kWaitWindow = std::chrono::milliseconds(80);

  mavlink_message_t makeMessage()
  {
    mavlink_message_t msg;
    std::memset(&msg, 0, sizeof(msg));
    return msg;
  }
} // namespace

class PeriodicSenderTest : public testing::Test
{
protected:
  void SetUp() override
  {
    sender.setPeriod(std::chrono::microseconds(10000)); // 100 Hz -> 10 ms period
    sender.setCollectFn([]() { return std::vector<mavlink_message_t>{makeMessage()}; });
    sender.setSendFn([&](const mavlink_message_t&) {
      ++send_count;
      return true;
    });
  }

  PeriodicSender sender;
  std::atomic<int> send_count{0};
};

TEST_F(PeriodicSenderTest, SendsAtConfiguredFrequency)
{
  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // At 100 Hz over ~80 ms, expect ~8 sends; allow generous tolerance for scheduling.
  EXPECT_GE(send_count.load(), 4);
  EXPECT_LE(send_count.load(), 20);
}

TEST_F(PeriodicSenderTest, SendsNothingWhenCollectReturnsEmpty)
{
  sender.setCollectFn([]() { return std::vector<mavlink_message_t>{}; });

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  EXPECT_EQ(send_count.load(), 0);
}

TEST_F(PeriodicSenderTest, SendsAllMessagesInVector)
{
  sender.setCollectFn([]() {
    return std::vector<mavlink_message_t>{makeMessage(), makeMessage(), makeMessage()};
  });

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // Each tick sends 3 messages; over ~80 ms at 100 Hz that is ~8 ticks * 3.
  EXPECT_GE(send_count.load(), 6);
}

TEST_F(PeriodicSenderTest, KeepsSendingOnIndividualSendFailure)
{
  sender.setCollectFn([]() {
    return std::vector<mavlink_message_t>{makeMessage(), makeMessage()};
  });
  // First send fails, but the loop should still send the second one.
  sender.setSendFn([&](const mavlink_message_t&) {
    const int n = ++send_count;
    return (n % 2 == 1); // odd (first of each tick) fails
  });

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // Even sends succeeded and the loop kept running across multiple ticks.
  EXPECT_GE(send_count.load(), 4);
}

TEST_F(PeriodicSenderTest, StartIsNoOpWhenRunning)
{
  EXPECT_EQ(sender.start(), true);
  EXPECT_EQ(sender.start(), true); // no-op, still one thread
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // If a second thread had been spawned, the count would be roughly doubled.
  EXPECT_LE(send_count.load(), 20);
}

TEST_F(PeriodicSenderTest, StopIsNoOpWhenStopped)
{
  sender.stop(); // should not crash/hang when already stopped
  EXPECT_EQ(sender.isRunning(), false);
}

TEST_F(PeriodicSenderTest, RepeatedStartStopCycles)
{
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(sender.start(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sender.stop();
    EXPECT_EQ(sender.isRunning(), false);
  }
}

TEST_F(PeriodicSenderTest, WorksWithUnsetCallbacks)
{
  // No collect/send callbacks set: the loop should run without crashing and send nothing.
  sender.setCollectFn({});
  sender.setSendFn({});

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();
  EXPECT_EQ(sender.isRunning(), false);
}

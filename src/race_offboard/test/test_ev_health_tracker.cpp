#include <gtest/gtest.h>

#include <string>

#include "race_offboard/ev_health_tracker.hpp"

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

race_offboard::EvHealthTracker makeTracker(double required_s = 7.5, double freshness_s = 1.0)
{
  race_offboard::EvHealthTracker tracker;
  tracker.configure(required_s, freshness_s);
  return tracker;
}

// Feed HEALTHY at 2 Hz, which is inside the 1 s freshness window.
std::int64_t feedHealthy(
  race_offboard::EvHealthTracker & tracker, std::int64_t start_ns, double duration_s)
{
  std::int64_t now = start_ns;
  const std::int64_t step = kSecond / 2;
  const std::int64_t end = start_ns + static_cast<std::int64_t>(duration_s * kSecond);
  while (now <= end) {
    tracker.update("HEALTHY", now);
    now += step;
  }
  return now - step;
}

}  // namespace

TEST(EvHealthTracker, StartsUnknownAndNotReady)
{
  auto tracker = makeTracker();
  EXPECT_EQ(tracker.state(), race_offboard::EvHealthState::Unknown);
  EXPECT_FALSE(tracker.ready(0));
  EXPECT_FALSE(tracker.isFresh(0));
  std::string reason;
  EXPECT_TRUE(tracker.unsafeReason(0, &reason));
  EXPECT_NE(reason.find("has not been received"), std::string::npos);
}

TEST(EvHealthTracker, NotReadyBeforeRequiredContinuousDuration)
{
  auto tracker = makeTracker(7.5, 1.0);
  const std::int64_t last = feedHealthy(tracker, kSecond, 7.0);
  EXPECT_EQ(tracker.state(), race_offboard::EvHealthState::Healthy);
  EXPECT_TRUE(tracker.isFresh(last));
  EXPECT_FALSE(tracker.ready(last)) << "7.0 s of HEALTHY must not satisfy a 7.5 s gate";
}

TEST(EvHealthTracker, ReadyAfterRequiredContinuousDuration)
{
  auto tracker = makeTracker(7.5, 1.0);
  const std::int64_t last = feedHealthy(tracker, kSecond, 8.0);
  EXPECT_TRUE(tracker.ready(last));
  std::string reason;
  EXPECT_FALSE(tracker.unsafeReason(last, &reason));
}

// The core pre-arm requirement: a SUSPECT blip restarts the whole 7.5 s count.
TEST(EvHealthTracker, SuspectResetsTheContinuousHealthyTimer)
{
  auto tracker = makeTracker(7.5, 1.0);
  std::int64_t now = feedHealthy(tracker, kSecond, 7.0);

  now += kSecond / 2;
  tracker.update("SUSPECT", now);
  EXPECT_EQ(tracker.state(), race_offboard::EvHealthState::Suspect);
  EXPECT_FALSE(tracker.ready(now));
  EXPECT_DOUBLE_EQ(tracker.healthyDurationS(now), 0.0);

  // Re-establishing HEALTHY starts from zero, so the old 7 s is not credited.
  const std::int64_t resumed = feedHealthy(tracker, now + kSecond / 2, 7.0);
  EXPECT_FALSE(tracker.ready(resumed))
    << "the timer must restart after SUSPECT, not resume";

  const std::int64_t completed = feedHealthy(tracker, resumed + kSecond / 2, 1.0);
  EXPECT_TRUE(tracker.ready(completed));
}

TEST(EvHealthTracker, GapLongerThanFreshnessRestartsContinuity)
{
  auto tracker = makeTracker(7.5, 1.0);
  std::int64_t now = feedHealthy(tracker, kSecond, 7.0);

  // No message for 3 s, then HEALTHY again: the stream was interrupted even
  // though no SUSPECT was ever published.
  now += 3 * kSecond;
  tracker.update("HEALTHY", now);
  EXPECT_LT(tracker.healthyDurationS(now), 0.1);
  EXPECT_FALSE(tracker.ready(now));
}

TEST(EvHealthTracker, StaleStatusIsNotFreshAndBlocksReady)
{
  auto tracker = makeTracker(7.5, 1.0);
  const std::int64_t last = feedHealthy(tracker, kSecond, 8.0);
  ASSERT_TRUE(tracker.ready(last));

  const std::int64_t stale = last + 2 * kSecond;
  EXPECT_FALSE(tracker.isFresh(stale));
  EXPECT_FALSE(tracker.ready(stale));
  std::string reason;
  EXPECT_TRUE(tracker.unsafeReason(stale, &reason));
  EXPECT_NE(reason.find("timeout"), std::string::npos);
}

TEST(EvHealthTracker, FaultIsReportedForInFlightGuard)
{
  auto tracker = makeTracker();
  const std::int64_t last = feedHealthy(tracker, kSecond, 8.0);
  EXPECT_FALSE(tracker.faulted());

  tracker.update("FAULT", last + kSecond / 2);
  EXPECT_TRUE(tracker.faulted());
  EXPECT_FALSE(tracker.ready(last + kSecond / 2));
}

// SUSPECT alone must not land the aircraft: that matches the flown behaviour.
TEST(EvHealthTracker, SuspectDoesNotCountAsFault)
{
  auto tracker = makeTracker();
  tracker.update("SUSPECT", kSecond);
  EXPECT_FALSE(tracker.faulted());
  EXPECT_EQ(tracker.state(), race_offboard::EvHealthState::Suspect);
}

TEST(EvHealthTracker, UnrecognisedStatusIsTreatedAsFault)
{
  auto tracker = makeTracker();
  tracker.update("banana", kSecond);
  EXPECT_TRUE(tracker.faulted());
  std::string reason;
  EXPECT_TRUE(tracker.unsafeReason(kSecond, &reason));
  EXPECT_NE(reason.find("invalid"), std::string::npos);
}

TEST(EvHealthTracker, ParsingIgnoresCaseAndSurroundingWhitespace)
{
  bool recognised = false;
  EXPECT_EQ(
    race_offboard::parseEvHealthState(" healthy \n", &recognised),
    race_offboard::EvHealthState::Healthy);
  EXPECT_TRUE(recognised);
  EXPECT_EQ(
    race_offboard::parseEvHealthState("Fault", &recognised),
    race_offboard::EvHealthState::Fault);
  EXPECT_TRUE(recognised);
  race_offboard::parseEvHealthState("", &recognised);
  EXPECT_FALSE(recognised);
}

TEST(EvHealthTracker, UpdateSignalsStateChangeForLogging)
{
  auto tracker = makeTracker();
  EXPECT_TRUE(tracker.update("HEALTHY", kSecond)) << "first sample is a change";
  EXPECT_FALSE(tracker.update("HEALTHY", kSecond + kSecond / 2))
    << "steady HEALTHY should not spam the log";
  EXPECT_TRUE(tracker.update("SUSPECT", kSecond * 2)) << "transition must be reported";
}

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "race_offboard/px4_local_z_reset_tracker.hpp"

TEST(Px4LocalZResetTracker, FirstCounterOnlyEstablishesBaseline)
{
  race_offboard::Px4LocalZResetTracker tracker;
  const auto observation = tracker.observe(2U, 0.37);

  EXPECT_EQ(observation.decision, race_offboard::Px4LocalZResetDecision::Initialized);
  EXPECT_DOUBLE_EQ(observation.delta_z_ned, 0.0);
  EXPECT_DOUBLE_EQ(tracker.cumulativeDeltaZNed(), 0.0);
}

TEST(Px4LocalZResetTracker, AppliesEachCounterChangeExactlyOnce)
{
  race_offboard::Px4LocalZResetTracker tracker;
  tracker.observe(2U, 0.0);

  const auto reset = tracker.observe(3U, 0.370641);
  EXPECT_EQ(reset.decision, race_offboard::Px4LocalZResetDecision::Applied);
  EXPECT_NEAR(reset.delta_z_ned, 0.370641, 1e-12);
  EXPECT_NEAR(reset.cumulative_delta_z_ned, 0.370641, 1e-12);

  const auto duplicate = tracker.observe(3U, 0.370641);
  EXPECT_EQ(duplicate.decision, race_offboard::Px4LocalZResetDecision::Duplicate);
  EXPECT_NEAR(tracker.cumulativeDeltaZNed(), 0.370641, 1e-12);
}

TEST(Px4LocalZResetTracker, RejectsNonFiniteDeltaWithoutLosingTheResetEvent)
{
  race_offboard::Px4LocalZResetTracker tracker;
  tracker.observe(7U, 0.0);

  const auto invalid = tracker.observe(8U, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(invalid.decision, race_offboard::Px4LocalZResetDecision::RejectedNonFinite);
  EXPECT_DOUBLE_EQ(tracker.cumulativeDeltaZNed(), 0.0);

  const auto recovered = tracker.observe(8U, -0.12);
  EXPECT_EQ(recovered.decision, race_offboard::Px4LocalZResetDecision::Applied);
  EXPECT_NEAR(tracker.cumulativeDeltaZNed(), -0.12, 1e-12);
}

TEST(Px4LocalZResetTracker, HandlesTheUint8CounterWrap)
{
  race_offboard::Px4LocalZResetTracker tracker;
  tracker.observe(255U, 0.0);

  const auto reset = tracker.observe(0U, 0.05);
  EXPECT_EQ(reset.decision, race_offboard::Px4LocalZResetDecision::Applied);
  EXPECT_NEAR(tracker.cumulativeDeltaZNed(), 0.05, 1e-12);
}

TEST(Px4LocalZResetTracker, RebasesCachedAndRepeatedTargetsInTheCorrectDirection)
{
  // From the 2026-08-17 hover: PX4 reset delta_z=+0.370641. The old fixed
  // local target -0.670 must become about -0.299, not remain -0.670.
  EXPECT_NEAR(
    race_offboard::rebaseLocalNedZ(-0.670, 0.370641), -0.299359, 1e-12);
  // The same physical map-frame target needs the opposite ENU translation.
  EXPECT_NEAR(race_offboard::rebaseMapToLocalEnuZ(0.0, 0.370641), -0.370641, 1e-12);
}

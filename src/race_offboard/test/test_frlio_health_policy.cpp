#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "race_offboard/frlio_health_policy.hpp"

// The core requirement: a stale LiDAR posterior must stop planning WITHOUT
// taking the PX4 external-vision stream away.
TEST(FrlioHealthPolicy, StaleLidarBlocksPlanningButKeepsEvFusion)
{
  EXPECT_FALSE(race_offboard::frlioStatusAllowsPlanning("FAULT_STALE_LIDAR"));
  EXPECT_TRUE(race_offboard::frlioStatusAllowsEvFusion("FAULT_STALE_LIDAR"));
}

TEST(FrlioHealthPolicy, HealthyAndDegradedBothAllowPlanningAndFusion)
{
  for (const std::string status : {"HEALTHY", "SUSPECT_STALE_LIDAR"}) {
    EXPECT_TRUE(race_offboard::frlioStatusAllowsPlanning(status)) << status;
    EXPECT_TRUE(race_offboard::frlioStatusAllowsEvFusion(status)) << status;
  }
}

TEST(FrlioHealthPolicy, OnlyStateUnusableClassStatusesRemoveEv)
{
  for (const std::string status :
    {"FAULT_STATE_UNUSABLE", "UNIT_UNCONFIRMED", "WAITING_FOR_LIDAR", "DISABLED"})
  {
    EXPECT_FALSE(race_offboard::frlioStatusAllowsPlanning(status)) << status;
    EXPECT_FALSE(race_offboard::frlioStatusAllowsEvFusion(status)) << status;
  }
}

// An unknown status is a reason to stop planning, never a reason to interrupt
// EV: interrupting EV is strictly more dangerous than continuing to fuse a
// covariance-inflated pose while PX4 EKF2 applies its own gating.
TEST(FrlioHealthPolicy, UnknownStatusStopsPlanningButNotFusion)
{
  EXPECT_FALSE(race_offboard::frlioStatusAllowsPlanning("SOMETHING_NEW"));
  EXPECT_TRUE(race_offboard::frlioStatusAllowsEvFusion("SOMETHING_NEW"));
  EXPECT_FALSE(race_offboard::frlioStatusAllowsPlanning(""));
  EXPECT_TRUE(race_offboard::frlioStatusAllowsEvFusion(""));
}

TEST(FrlioHealthPolicy, HoldReleaseRequiresUsableStateStabilityAndLowSpeed)
{
  // Not usable yet: never releases.
  EXPECT_FALSE(race_offboard::frlioHoldReleaseReady(true, false, 5.0, 1.0, 0.0, 0.05));
  // Usable but not stable long enough.
  EXPECT_FALSE(race_offboard::frlioHoldReleaseReady(true, true, 0.5, 1.0, 0.0, 0.05));
  // Stable long enough but still moving: resuming would hand the planner a
  // moving start state.
  EXPECT_FALSE(race_offboard::frlioHoldReleaseReady(true, true, 2.0, 1.0, 0.40, 0.05));
  // All conditions met.
  EXPECT_TRUE(race_offboard::frlioHoldReleaseReady(true, true, 2.0, 1.0, 0.01, 0.05));
  // No hold to release.
  EXPECT_FALSE(race_offboard::frlioHoldReleaseReady(false, true, 2.0, 1.0, 0.01, 0.05));
}

TEST(FrlioHealthPolicy, HoldReleaseRejectsNonFiniteInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(race_offboard::frlioHoldReleaseReady(true, true, nan, 1.0, 0.0, 0.05));
  EXPECT_FALSE(race_offboard::frlioHoldReleaseReady(true, true, 2.0, 1.0, nan, 0.05));
}

// Planner HOLD must never stop the Offboard setpoint stream: PX4 drops out of
// OFFBOARD after roughly 0.5 s of silence, which would turn a localization
// degradation into a mode loss.
TEST(FrlioHealthPolicy, ArmedOrOffboardAlwaysRequiresContinuedSetpoints)
{
  EXPECT_TRUE(race_offboard::holdStateMustKeepPublishingSetpoints(true, false));
  EXPECT_TRUE(race_offboard::holdStateMustKeepPublishingSetpoints(false, true));
  EXPECT_TRUE(race_offboard::holdStateMustKeepPublishingSetpoints(true, true));
  EXPECT_FALSE(race_offboard::holdStateMustKeepPublishingSetpoints(false, false));
}

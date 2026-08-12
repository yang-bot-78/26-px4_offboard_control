#include <gtest/gtest.h>

#include "race_super_planner_ros2/narrow_corridor_policy.hpp"

TEST(NarrowCorridorPolicy, FirstGateCentreIntervalIsOnlyEightPointFiveCentimetres)
{
  const race_super_planner_ros2::policy::CorridorInterval gate{
    -0.5665, 0.5365, 0.509};
  const auto safe = gate.safeCentreInterval();
  ASSERT_TRUE(safe.has_value());
  EXPECT_NEAR(-0.0575, safe->first, 1e-9);
  EXPECT_NEAR(0.0275, safe->second, 1e-9);
  EXPECT_NEAR(-0.015, gate.centreline(), 1e-9);
  EXPECT_TRUE(race_super_planner_ros2::policy::insideSafeCentreInterval(-0.015, gate));
  EXPECT_FALSE(race_super_planner_ros2::policy::insideSafeCentreInterval(0.075, gate));
  // Shared-bounds grid origin -1.30m at 0.05m planning resolution yields a
  // connected gate chain at -0.025m and +0.025m; the next right cell is unsafe.
  EXPECT_TRUE(race_super_planner_ros2::policy::insideSafeCentreInterval(-0.025, gate));
  EXPECT_TRUE(race_super_planner_ros2::policy::insideSafeCentreInterval(0.025, gate));
  EXPECT_FALSE(race_super_planner_ros2::policy::insideSafeCentreInterval(0.075, gate));
  EXPECT_NEAR(
    0.0275,
    race_super_planner_ros2::policy::clampToSafeCentreInterval(0.075, gate), 1e-9);
}

TEST(NarrowCorridorPolicy, RejectsPhysicallyTooNarrowPassage)
{
  const race_super_planner_ros2::policy::CorridorInterval too_narrow{
    -0.5425, 0.4735, 0.509};
  EXPECT_FALSE(too_narrow.safeCentreInterval().has_value());
}

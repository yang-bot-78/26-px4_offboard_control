#include <gtest/gtest.h>

#include "race_super_planner_ros2/takeoff_path_policy.hpp"

namespace policy = race_super_planner_ros2;

TEST(TakeoffPathPolicy, FreezesGlobalOnlyPathBelowMinimumHeight)
{
  EXPECT_TRUE(policy::shouldFreezeGlobalPathDuringTakeoff(true, false, 0.23, 0.50));
}

TEST(TakeoffPathPolicy, ReleasesPathAtMinimumHeight)
{
  EXPECT_FALSE(policy::shouldFreezeGlobalPathDuringTakeoff(true, false, 0.50, 0.50));
  EXPECT_FALSE(policy::shouldFreezeGlobalPathDuringTakeoff(true, false, 0.70, 0.50));
}

TEST(TakeoffPathPolicy, DoesNotAffectSuperControlledMode)
{
  EXPECT_FALSE(policy::shouldFreezeGlobalPathDuringTakeoff(false, false, 0.10, 0.50));
}

TEST(TakeoffPathPolicy, InFlightHeightExcursionDoesNotReenterTakeoffFreeze)
{
  EXPECT_FALSE(policy::shouldFreezeGlobalPathDuringTakeoff(true, true, 0.23, 0.50));
}

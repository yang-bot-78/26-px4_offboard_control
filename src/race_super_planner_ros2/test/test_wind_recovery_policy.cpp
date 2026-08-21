#include <gtest/gtest.h>

#include "race_super_planner_ros2/wind_recovery_policy.hpp"

namespace policy = race_super_planner_ros2::wind_recovery;

TEST(WindRecoveryPolicy, UsesTheApprovedThreeStageLimits)
{
  const auto base = policy::limitForStage(0);
  EXPECT_DOUBLE_EQ(base.max_velocity_mps, 0.8);
  EXPECT_DOUBLE_EQ(base.max_acceleration_mps2, 1.0);

  const auto first_boost = policy::limitForStage(1);
  EXPECT_DOUBLE_EQ(first_boost.max_velocity_mps, 1.2);
  EXPECT_DOUBLE_EQ(first_boost.max_acceleration_mps2, 1.5);

  const auto final_boost = policy::limitForStage(2);
  EXPECT_DOUBLE_EQ(final_boost.max_velocity_mps, 1.5);
  EXPECT_DOUBLE_EQ(final_boost.max_acceleration_mps2, 2.0);
}

TEST(WindRecoveryPolicy, ClampsOutOfRangeStagesAndRequiresStrictProgress)
{
  EXPECT_DOUBLE_EQ(policy::limitForStage(-1).max_velocity_mps, 0.8);
  EXPECT_DOUBLE_EQ(policy::limitForStage(99).max_velocity_mps, 1.5);
  EXPECT_FALSE(policy::hasRecovered(0.15, 0.15));
  EXPECT_TRUE(policy::hasRecovered(0.151, 0.15));
}

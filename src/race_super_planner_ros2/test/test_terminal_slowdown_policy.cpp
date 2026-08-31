#include <limits>

#include <gtest/gtest.h>

#include "race_super_planner_ros2/terminal_slowdown_policy.hpp"

namespace policy = race_super_planner_ros2::terminal_slowdown;

TEST(TerminalSlowdownPolicy, KeepsNominalLimitBeforeSlowdownZone)
{
  const policy::Profile profile{true, 1.20, 0.10};
  EXPECT_DOUBLE_EQ(policy::scale(2.0, profile), 1.0);
  EXPECT_DOUBLE_EQ(policy::scaledLimit(0.8, 1.20, profile), 0.8);
}

TEST(TerminalSlowdownPolicy, DecreasesMonotonicallyTowardFinalGoal)
{
  const policy::Profile profile{true, 1.20, 0.10};
  const double far_limit = policy::scaledLimit(0.8, 1.0, profile);
  const double middle_limit = policy::scaledLimit(0.8, 0.6, profile);
  const double near_limit = policy::scaledLimit(0.8, 0.2, profile);
  EXPECT_GT(far_limit, middle_limit);
  EXPECT_GT(middle_limit, near_limit);
  EXPECT_GT(near_limit, 0.0);
  EXPECT_LE(far_limit, 0.8);
}

TEST(TerminalSlowdownPolicy, CommandsZeroInsideStopDistance)
{
  const policy::Profile profile{true, 1.20, 0.10};
  EXPECT_DOUBLE_EQ(policy::scaledLimit(0.8, 0.10, profile), 0.0);
  EXPECT_DOUBLE_EQ(policy::scaledLimit(0.8, 0.05, profile), 0.0);
}

TEST(TerminalSlowdownPolicy, InvalidEnabledProfileFailsSafe)
{
  const policy::Profile reversed{true, 0.10, 0.20};
  const policy::Profile non_finite{true, std::numeric_limits<double>::infinity(), 0.10};
  EXPECT_DOUBLE_EQ(policy::scaledLimit(0.8, 0.5, reversed), 0.0);
  EXPECT_DOUBLE_EQ(policy::scaledLimit(0.8, 0.5, non_finite), 0.0);
  EXPECT_DOUBLE_EQ(policy::scaledLimit(-1.0, 0.5, policy::Profile{false, 0.0, 0.0}), 0.0);
}

TEST(TerminalSlowdownPolicy, DisabledProfileLeavesNominalLimitUnchanged)
{
  const policy::Profile disabled{false, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(policy::scaledLimit(0.8, 0.01, disabled), 0.8);
}

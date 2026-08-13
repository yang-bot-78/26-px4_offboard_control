#include <gtest/gtest.h>

#include "race_offboard/global_planner_status_policy.hpp"

TEST(GlobalPlannerStatusPolicy, IgnoresStartupAndStaleGoalStatuses)
{
  EXPECT_FALSE(
    race_offboard::globalPlannerStatusIsFailure(
      false, true, 1, 1, "HOLD", "NO_PATH"));
  EXPECT_FALSE(
    race_offboard::globalPlannerStatusIsFailure(
      true, false, 1, 1, "HOLD", "NO_PATH"));
  EXPECT_FALSE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 2, 1, "HOLD", "NO_PATH"));
  EXPECT_FALSE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 1, 1, "IDLE", "NO_GOAL"));
}

TEST(GlobalPlannerStatusPolicy, RejectsFailuresForTheActiveGoal)
{
  EXPECT_TRUE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 4, 4, "HOLD", "NO_PATH"));
  EXPECT_TRUE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 4, 4, "HOLD", "PATH_TRACKING_ERROR"));
  EXPECT_TRUE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 4, 4, "BLOCKED_UNSAFE", "BLOCKED_UNSAFE"));
  EXPECT_TRUE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 4, 4, "HOLD", "START_OUTSIDE_PLANNING_GRID"));
  EXPECT_TRUE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 4, 4, "NO_MAP", "NO_MAP"));
}

TEST(GlobalPlannerStatusPolicy, HealthyFollowingStatusIsAllowed)
{
  EXPECT_FALSE(
    race_offboard::globalPlannerStatusIsFailure(
      true, true, 4, 4, "FOLLOWING", "GLOBAL_PATH_TO_EGO"));
  EXPECT_FALSE(
    race_offboard::globalPlannerFailureAppliesToGoal(
      true, true, 4, 5));
}

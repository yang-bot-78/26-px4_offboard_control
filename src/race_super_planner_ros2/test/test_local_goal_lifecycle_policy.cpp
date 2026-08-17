#include <limits>

#include <gtest/gtest.h>

#include "race_super_planner_ros2/local_goal_lifecycle_policy.hpp"

TEST(LocalGoalLifecycle, FinalApproachClosesShortPathTailOnce)
{
  EXPECT_TRUE(
    race_super_planner_ros2::shouldPublishFinalApproach(
      0.058, 0.20, 0.058, 0.05, false, false));
  EXPECT_FALSE(
    race_super_planner_ros2::shouldPublishFinalApproach(
      0.058, 0.20, 0.058, 0.05, false, true));
  EXPECT_FALSE(
    race_super_planner_ros2::shouldPublishFinalApproach(
      0.058, 0.20, 0.058, 0.05, true, false));
}

TEST(LocalGoalLifecycle, OneTransientStatusCannotHideObjectiveStall)
{
  EXPECT_TRUE(
    race_super_planner_ros2::localPlanningStalled(
      true, true, false, 1.1, 2.0, 1.0));
  EXPECT_TRUE(
    race_super_planner_ros2::localPlanningStalled(
      true, false, true, 1.1, 2.0, 1.0));
  EXPECT_FALSE(
    race_super_planner_ros2::localPlanningStalled(
      true, true, false, 0.6, 2.0, 1.0));
}

TEST(LocalGoalLifecycle, PlannerUnusablePausesObjectiveStall)
{
  EXPECT_FALSE(
    race_super_planner_ros2::localPlanningStalled(
      true, true, true, 30.0, -10.0, 3.0, false));
  EXPECT_TRUE(
    race_super_planner_ros2::localPlanningStalled(
      true, true, true, 30.0, -10.0, 3.0, true));
}

TEST(LocalGoalLifecycle, LocalGoalStaysFixedUntilBridgeResumesCommands)
{
  EXPECT_TRUE(
    race_super_planner_ros2::shouldHoldLocalGoalForPlannerRecovery(true, false));
  EXPECT_TRUE(
    race_super_planner_ros2::shouldHoldLocalGoalForPlannerRecovery(false, true));
  EXPECT_FALSE(
    race_super_planner_ros2::shouldHoldLocalGoalForPlannerRecovery(false, false));
}

TEST(LocalGoalLifecycle, FrlioRecoveryWaitsForBridgeThenRequiresFreshGoal)
{
  EXPECT_FALSE(
    race_super_planner_ros2::shouldArmFreshLocalGoalForFrlioRecovery(
      true, false, false));
  EXPECT_FALSE(
    race_super_planner_ros2::shouldArmFreshLocalGoalForFrlioRecovery(
      true, true, true));
  EXPECT_TRUE(
    race_super_planner_ros2::shouldArmFreshLocalGoalForFrlioRecovery(
      true, true, false));
}

TEST(LocalGoalLifecycle, NewerValidatedTrajectoriesDoNotRestartRecoveryWindow)
{
  EXPECT_TRUE(race_super_planner_ros2::shouldStartRecoveryConfirmation(39, 38, false));
  EXPECT_FALSE(race_super_planner_ros2::shouldStartRecoveryConfirmation(40, 38, true));
  EXPECT_FALSE(race_super_planner_ros2::shouldStartRecoveryConfirmation(38, 38, false));
}

TEST(LocalGoalLifecycle, CurrentValidatedTrajectoryMayRecoverItsCommandStream)
{
  EXPECT_TRUE(
    race_super_planner_ros2::shouldConfirmCurrentTrajectoryRecovery(
      true, 38, 0.4));
  EXPECT_FALSE(
    race_super_planner_ros2::shouldConfirmCurrentTrajectoryRecovery(
      true, 38, 0.0));
  EXPECT_FALSE(
    race_super_planner_ros2::shouldConfirmCurrentTrajectoryRecovery(
      false, 38, 0.4));
}

#include <gtest/gtest.h>

#include "race_super_planner_ros2/global_grid_policy.hpp"

namespace policy = race_super_planner_ros2;

TEST(GlobalGridPolicy, RegressionGoalUsesFullSharedBoundsGrid)
{
  const auto grid = policy::makeSharedBoundsGrid(-0.85, 11.90, -1.30, 12.40, 0.15);
  EXPECT_TRUE(grid.contains(-0.249, -0.225));
  EXPECT_TRUE(grid.contains(10.670, 3.646));
  EXPECT_EQ(86, grid.width);
  EXPECT_EQ(92, grid.height);

  const auto selection = policy::selectPlanningGoal(
    true, -0.249, -0.225, 10.670, 3.646, 9.0);
  EXPECT_FALSE(selection.clipped);
  EXPECT_DOUBLE_EQ(10.670, selection.x);
  EXPECT_DOUBLE_EQ(3.646, selection.y);
  EXPECT_NEAR(10.670, selection.x, 0.15);
  EXPECT_NEAR(3.646, selection.y, 0.15);
}

TEST(GlobalGridPolicy, LocalModeKeepsDistanceLimitedSubgoal)
{
  const auto selection = policy::selectPlanningGoal(
    false, -0.249, -0.225, 10.670, 3.646, 9.0);
  EXPECT_TRUE(selection.clipped);
  EXPECT_NEAR(8.233, selection.x, 0.002);
  EXPECT_NEAR(2.782, selection.y, 0.001);
}

TEST(GlobalGridPolicy, ShortGlobalGoalIsNotModified)
{
  const auto selection = policy::selectPlanningGoal(
    true, 0.0, 0.0, 2.0, 1.0, 9.0);
  EXPECT_FALSE(selection.clipped);
  EXPECT_DOUBLE_EQ(2.0, selection.x);
  EXPECT_DOUBLE_EQ(1.0, selection.y);
}

TEST(GlobalGridPolicy, BoundaryCellsAreIncluded)
{
  const auto grid = policy::makeSharedBoundsGrid(-0.85, 11.90, -1.30, 12.40, 0.15);
  EXPECT_TRUE(grid.contains(-0.85, -1.30));
  EXPECT_TRUE(grid.contains(11.90, 12.40));
  EXPECT_FALSE(policy::insideSharedBounds(11.91, 3.0, -0.85, 11.90, -1.30, 12.40));
  EXPECT_FALSE(policy::insideSharedBounds(3.0, 12.41, -0.85, 11.90, -1.30, 12.40));
  EXPECT_FALSE(policy::insideSharedBounds(-0.86, 3.0, -0.85, 11.90, -1.30, 12.40));
  EXPECT_FALSE(policy::insideSharedBounds(3.0, -1.31, -0.85, 11.90, -1.30, 12.40));
}

TEST(GlobalGridPolicy, AstarDoesNotUseOuterCellWhoseCentreExceedsGeofence)
{
  const auto grid = policy::makeSharedBoundsGrid(-0.85, 11.90, -1.30, 12.40, 0.05);
  // x=11.90 maps to the included final index, but that cell centre is 11.925.
  EXPECT_TRUE(grid.contains(11.90, 3.0));
  EXPECT_TRUE(
    policy::cellCenterInsideSharedBounds(
      grid.width - 2, 0, grid, -0.85, 11.90, -1.30, 12.40));
  EXPECT_FALSE(
    policy::cellCenterInsideSharedBounds(
      grid.width - 1, 0, grid, -0.85, 11.90, -1.30, 12.40));
}

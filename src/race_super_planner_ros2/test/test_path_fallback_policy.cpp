#include <gtest/gtest.h>

#include "race_super_planner_ros2/path_fallback_policy.hpp"

namespace policy = race_super_planner_ros2;

TEST(PathFallbackPolicy, SelectsSmoothedWhenBothPathsAreSafe)
{
  EXPECT_EQ(
    policy::PathSource::SMOOTHED,
    policy::selectValidatedPath({}, {}));
}

TEST(PathFallbackPolicy, FallsBackToRawWhenSmoothedPathIsOutOfBounds)
{
  policy::PathValidationResult smoothed;
  smoothed.failure = policy::PathValidationFailure::OUT_OF_BOUNDS;
  EXPECT_EQ(
    policy::PathSource::RAW_ASTAR,
    policy::selectValidatedPath(smoothed, {}));
}

TEST(PathFallbackPolicy, FallsBackToRawWhenSmoothedPathCollides)
{
  policy::PathValidationResult smoothed;
  smoothed.failure = policy::PathValidationFailure::COLLISION;
  EXPECT_EQ(
    policy::PathSource::RAW_ASTAR,
    policy::selectValidatedPath(smoothed, {}));
}

TEST(PathFallbackPolicy, RejectsWhenNeitherPathIsSafe)
{
  policy::PathValidationResult smoothed;
  smoothed.failure = policy::PathValidationFailure::OUT_OF_BOUNDS;
  policy::PathValidationResult raw;
  raw.failure = policy::PathValidationFailure::COLLISION;
  EXPECT_EQ(policy::PathSource::NONE, policy::selectValidatedPath(smoothed, raw));
}

TEST(PathFallbackPolicy, RawSelectionIsAContinuePlanningOutcome)
{
  policy::PathValidationResult smoothed;
  smoothed.failure = policy::PathValidationFailure::COLLISION;
  const auto source = policy::selectValidatedPath(smoothed, {});
  EXPECT_EQ(policy::PathSource::RAW_ASTAR, source);
  EXPECT_NE(policy::PathSource::NONE, source);
}

#include <gtest/gtest.h>

#include "race_super_planner_ros2/pending_path_gate_policy.hpp"

namespace
{
using race_super_planner_ros2::PendingPathGateDecision;
using race_super_planner_ros2::PendingPathGateInput;
using race_super_planner_ros2::decidePendingPathGate;

PendingPathGateInput freshInput()
{
  PendingPathGateInput input;
  input.pending_goal_id = 7;
  input.current_goal_id = 7;
  input.plan_completion_odom_generation = 20;
  input.current_odom_generation = 21;
  input.plan_completion_odom_source_stamp_ns = 1000;
  input.current_odom_source_stamp_ns = 1001;
  input.projection_valid = true;
  input.maximum_path_cross_track_error_m = 0.15;
  input.maximum_retries = 1;
  return input;
}

TEST(PendingPathGatePolicy, WaitsForCallbackAfterPlanCompletion)
{
  auto input = freshInput();
  input.current_odom_generation = input.plan_completion_odom_generation;
  EXPECT_EQ(
    PendingPathGateDecision::WAIT_FOR_FRESH_ODOM,
    decidePendingPathGate(input));
}

TEST(PendingPathGatePolicy, WaitsForNewerSourceTimestamp)
{
  auto input = freshInput();
  input.current_odom_source_stamp_ns = input.plan_completion_odom_source_stamp_ns;
  EXPECT_EQ(
    PendingPathGateDecision::WAIT_FOR_FRESH_ODOM,
    decidePendingPathGate(input));
}

TEST(PendingPathGatePolicy, CommitsBasedOnWholePathDistanceNotOldStartDistance)
{
  auto input = freshInput();
  // The old start may be 0.60 m behind the aircraft. It is deliberately not
  // an input to this policy; only distance to the complete route is gated.
  input.path_cross_track_error_m = 0.03;
  EXPECT_EQ(
    PendingPathGateDecision::COMMIT_TRIMMED_PATH,
    decidePendingPathGate(input));
}

TEST(PendingPathGatePolicy, RejectsRouteOutsideFirstCommitGate)
{
  auto input = freshInput();
  input.path_cross_track_error_m = 0.16;
  EXPECT_EQ(
    PendingPathGateDecision::RETRY_FROM_LATEST_ODOM,
    decidePendingPathGate(input));
}

TEST(PendingPathGatePolicy, HoldsAfterExactlyOneRetry)
{
  auto input = freshInput();
  input.path_cross_track_error_m = 0.16;
  input.completed_retries = 1;
  EXPECT_EQ(
    PendingPathGateDecision::HOLD_RETRY_EXHAUSTED,
    decidePendingPathGate(input));
}

TEST(PendingPathGatePolicy, DiscardsResultForChangedGoal)
{
  auto input = freshInput();
  input.current_goal_id = 8;
  EXPECT_EQ(
    PendingPathGateDecision::DISCARD_GOAL_CHANGED,
    decidePendingPathGate(input));
}

TEST(PendingPathGatePolicy, InvalidProjectionNeverCommits)
{
  auto input = freshInput();
  input.projection_valid = false;
  EXPECT_EQ(
    PendingPathGateDecision::RETRY_FROM_LATEST_ODOM,
    decidePendingPathGate(input));
}
}  // namespace

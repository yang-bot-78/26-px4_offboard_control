#pragma once

#include <cstdint>

namespace race_super_planner_ros2
{

enum class PendingPathGateDecision
{
  WAIT_FOR_FRESH_ODOM,
  DISCARD_GOAL_CHANGED,
  COMMIT_TRIMMED_PATH,
  RETRY_FROM_LATEST_ODOM,
  HOLD_RETRY_EXHAUSTED
};

struct PendingPathGateInput
{
  uint64_t pending_goal_id{0};
  uint64_t current_goal_id{0};
  uint64_t plan_completion_odom_generation{0};
  uint64_t current_odom_generation{0};
  int64_t plan_completion_odom_source_stamp_ns{0};
  int64_t current_odom_source_stamp_ns{0};
  bool projection_valid{false};
  double path_cross_track_error_m{0.0};
  double maximum_path_cross_track_error_m{0.15};
  int completed_retries{0};
  int maximum_retries{1};
};

inline bool hasPostPlanFreshOdom(const PendingPathGateInput & input)
{
  if (input.current_odom_generation <= input.plan_completion_odom_generation) {
    return false;
  }
  if (input.plan_completion_odom_source_stamp_ns > 0 &&
    input.current_odom_source_stamp_ns > 0)
  {
    return input.current_odom_source_stamp_ns >
           input.plan_completion_odom_source_stamp_ns;
  }
  return true;
}

inline PendingPathGateDecision decidePendingPathGate(const PendingPathGateInput & input)
{
  if (input.pending_goal_id != input.current_goal_id) {
    return PendingPathGateDecision::DISCARD_GOAL_CHANGED;
  }
  if (!hasPostPlanFreshOdom(input)) {
    return PendingPathGateDecision::WAIT_FOR_FRESH_ODOM;
  }

  const bool path_is_current = input.projection_valid &&
    input.path_cross_track_error_m <= input.maximum_path_cross_track_error_m;
  if (path_is_current) {
    return PendingPathGateDecision::COMMIT_TRIMMED_PATH;
  }
  if (input.completed_retries < input.maximum_retries) {
    return PendingPathGateDecision::RETRY_FROM_LATEST_ODOM;
  }
  return PendingPathGateDecision::HOLD_RETRY_EXHAUSTED;
}

}  // namespace race_super_planner_ros2

#pragma once

#include <cmath>
#include <cstdint>

namespace race_super_planner_ros2
{

inline bool shouldPublishFinalApproach(
  const double remaining_path, const double ordinary_update_distance,
  const double previous_goal_to_path_end, const double duplicate_tolerance,
  const bool final_goal_reached, const bool final_approach_already_published)
{
  return std::isfinite(remaining_path) && remaining_path >= 0.0 &&
         remaining_path < ordinary_update_distance &&
         previous_goal_to_path_end > duplicate_tolerance &&
         !final_goal_reached && !final_approach_already_published;
}

inline bool localPlanningStalled(
  const bool mission_active, const bool current_local_goal_reached,
  const bool goal_waiting_for_validated_trajectory,
  const double validated_trajectory_age_sec,
  const double trajectory_lease_remaining_sec,
  const double stall_timeout_sec)
{
  if (!mission_active || !std::isfinite(stall_timeout_sec) || stall_timeout_sec <= 0.0) {
    return false;
  }
  const bool trajectory_stale = !std::isfinite(validated_trajectory_age_sec) ||
    validated_trajectory_age_sec > stall_timeout_sec;
  const bool lease_expiring = !std::isfinite(trajectory_lease_remaining_sec) ||
    trajectory_lease_remaining_sec <= 0.0;
  return trajectory_stale &&
         (current_local_goal_reached || goal_waiting_for_validated_trajectory || lease_expiring);
}

inline bool shouldStartRecoveryConfirmation(
  const int64_t validated_trajectory_id,
  const int64_t recovery_required_after_trajectory_id,
  const bool confirmation_already_active)
{
  return !confirmation_already_active &&
         validated_trajectory_id > recovery_required_after_trajectory_id;
}

}  // namespace race_super_planner_ros2

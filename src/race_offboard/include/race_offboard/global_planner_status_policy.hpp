#pragma once

#include <cstdint>
#include <string>

namespace race_offboard
{

inline bool globalPlannerFailureAppliesToGoal(
  const bool have_status,
  const bool status_goal_active,
  const std::uint64_t status_goal_id,
  const std::uint64_t active_goal_id)
{
  return have_status && status_goal_active && active_goal_id != 0U &&
         status_goal_id == active_goal_id;
}

inline bool globalPlannerStatusIsFailure(
  const bool have_status,
  const bool status_goal_active,
  const std::uint64_t status_goal_id,
  const std::uint64_t active_goal_id,
  const std::string & mode,
  const std::string & reason)
{
  if (!globalPlannerFailureAppliesToGoal(
      have_status, status_goal_active, status_goal_id, active_goal_id))
  {
    return false;
  }

  return mode == "NO_ODOM" || mode == "NO_MAP" || mode == "BLOCKED_UNSAFE" ||
         reason.find("NO_PATH") != std::string::npos ||
         reason.find("BLOCKED") != std::string::npos ||
         reason.find("PATH_TRACKING_ERROR") != std::string::npos ||
         reason.find("FAULT_") != std::string::npos ||
         reason.find("START_OUTSIDE_PLANNING_GRID") != std::string::npos ||
         reason.find("NO_ODOM") != std::string::npos ||
         reason.find("NO_MAP") != std::string::npos ||
         reason.find("MAVROS_LINK_DOWN") != std::string::npos;
}

}  // namespace race_offboard

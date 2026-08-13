#pragma once

#include <cmath>

namespace race_super_planner_ros2
{

inline bool shouldFreezeGlobalPathDuringTakeoff(
  bool global_only_mode, bool takeoff_handover_released,
  double current_height, double minimum_tracking_height)
{
  return global_only_mode && !takeoff_handover_released && std::isfinite(current_height) &&
         std::isfinite(minimum_tracking_height) &&
         current_height < minimum_tracking_height - 1.0e-3;
}

}  // namespace race_super_planner_ros2

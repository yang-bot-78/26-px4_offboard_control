#pragma once

#include <algorithm>

namespace race_super_planner_ros2::wind_recovery
{

struct MotionLimit
{
  double max_velocity_mps;
  double max_acceleration_mps2;
};

inline MotionLimit limitForStage(int stage)
{
  switch (std::clamp(stage, 0, 2)) {
    case 0:
      return {0.8, 1.0};
    case 1:
      return {1.2, 1.5};
    default:
      return {1.5, 2.0};
  }
}

inline bool hasRecovered(double progress_mps, double required_progress_mps)
{
  return progress_mps > required_progress_mps;
}

}  // namespace race_super_planner_ros2::wind_recovery

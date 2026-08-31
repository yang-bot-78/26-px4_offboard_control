#pragma once

#include <algorithm>
#include <cmath>

namespace race_super_planner_ros2::terminal_slowdown
{

struct Profile
{
  bool enabled;
  double slowdown_distance_m;
  double stop_distance_m;
};

inline double scale(double distance_to_final_m, const Profile & profile)
{
  if (!profile.enabled) {
    return 1.0;
  }
  if (!std::isfinite(distance_to_final_m) ||
    !std::isfinite(profile.slowdown_distance_m) ||
    !std::isfinite(profile.stop_distance_m) ||
    profile.stop_distance_m < 0.0 ||
    profile.slowdown_distance_m <= profile.stop_distance_m)
  {
    return 0.0;
  }
  return std::clamp(
    (distance_to_final_m - profile.stop_distance_m) /
    (profile.slowdown_distance_m - profile.stop_distance_m),
    0.0, 1.0);
}

inline double scaledLimit(
  double nominal_limit, double distance_to_final_m, const Profile & profile)
{
  if (!std::isfinite(nominal_limit) || nominal_limit < 0.0) {
    return 0.0;
  }
  return nominal_limit * scale(distance_to_final_m, profile);
}

}  // namespace race_super_planner_ros2::terminal_slowdown

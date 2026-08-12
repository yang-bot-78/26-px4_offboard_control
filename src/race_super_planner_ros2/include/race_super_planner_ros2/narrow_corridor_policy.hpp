#pragma once

#include <algorithm>
#include <optional>
#include <utility>

namespace race_super_planner_ros2::policy
{

struct CorridorInterval
{
  double lower_boundary{0.0};
  double upper_boundary{0.0};
  double required_clearance{0.0};

  std::optional<std::pair<double, double>> safeCentreInterval() const
  {
    const double lower = lower_boundary + required_clearance;
    const double upper = upper_boundary - required_clearance;
    if (lower > upper) {return std::nullopt;}
    return std::make_pair(lower, upper);
  }

  double centreline() const {return 0.5 * (lower_boundary + upper_boundary);}
};

inline bool insideSafeCentreInterval(double coordinate, const CorridorInterval & corridor)
{
  const auto interval = corridor.safeCentreInterval();
  return interval.has_value() && coordinate >= interval->first && coordinate <= interval->second;
}

inline double clampToSafeCentreInterval(double coordinate, const CorridorInterval & corridor)
{
  const auto interval = corridor.safeCentreInterval();
  if (!interval.has_value()) {return coordinate;}
  return std::clamp(coordinate, interval->first, interval->second);
}

}  // namespace race_super_planner_ros2::policy

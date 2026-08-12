#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "race_ego_bridge/frame_utils.hpp"

namespace race_ego_bridge
{

inline bool frameMatches(const std::string & actual, const std::string & required)
{
  return !actual.empty() && actual == required;
}

inline bool withinVectorLimit(const Vec3 & value, double limit, double tolerance = 1.0)
{
  return finite(value) && std::hypot(std::hypot(value.x, value.y), value.z) <= limit * tolerance;
}

inline bool timedOut(double age_sec, double timeout_sec)
{
  return !std::isfinite(age_sec) || age_sec < 0.0 || age_sec > timeout_sec;
}

inline double clampTrajectoryTime(double elapsed, double duration)
{
  if (!std::isfinite(elapsed) || !std::isfinite(duration) || duration < 0.0) {
    return 0.0;
  }
  return std::clamp(elapsed, 0.0, duration);
}

inline bool transitionContinuous(const Vec3 & previous, const Vec3 & next, double max_jump)
{
  if (!finite(previous) || !finite(next) || !std::isfinite(max_jump) || max_jump < 0.0) {
    return false;
  }
  return std::hypot(
    std::hypot(next.x - previous.x, next.y - previous.y), next.z - previous.z) <= max_jump;
}

struct ClearanceModeSelection
{
  bool accepted{false};
  double required_clearance{0.0};
};

inline ClearanceModeSelection selectTrajectoryClearance(
  bool constrained_marker, double requested_clearance,
  bool constrained_enabled, double normal_clearance,
  double constrained_clearance, double tolerance = 1.0e-6)
{
  ClearanceModeSelection result;
  if (!std::isfinite(normal_clearance) || normal_clearance <= 0.0 ||
    !std::isfinite(constrained_clearance) || constrained_clearance <= 0.0 ||
    constrained_clearance >= normal_clearance)
  {
    return result;
  }
  if (constrained_marker) {
    result.accepted = constrained_enabled && std::isfinite(requested_clearance) &&
      std::abs(requested_clearance - constrained_clearance) <= tolerance;
    result.required_clearance = constrained_clearance;
    return result;
  }
  // Zero is accepted for legacy normal messages. Any explicit request below
  // the normal bridge floor needs the constrained marker and is rejected.
  result.accepted = !std::isfinite(requested_clearance) || requested_clearance <= 0.0 ||
    requested_clearance + tolerance >= normal_clearance;
  result.required_clearance = normal_clearance;
  return result;
}

inline std::size_t spatialSubdivisions(double distance, double max_spacing)
{
  if (!std::isfinite(distance) || !std::isfinite(max_spacing) || distance < 0.0 ||
    max_spacing <= 0.0)
  {
    return 0;
  }
  return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(distance / max_spacing)));
}

inline std::size_t activeTrajectoryStartIndex(
  const std::vector<double> & sample_times, double elapsed, double history_sec)
{
  if (sample_times.empty() || !std::isfinite(elapsed) || !std::isfinite(history_sec)) {
    return 0;
  }
  const double start_time = std::max(0.0, elapsed - std::max(0.0, history_sec));
  auto first = std::lower_bound(sample_times.begin(), sample_times.end(), start_time);
  if (first == sample_times.end()) {
    return sample_times.size() - 1;
  }
  const std::size_t index = static_cast<std::size_t>(first - sample_times.begin());
  // Keep the sample immediately before the time boundary so the connecting
  // segment is checked as well.
  return index == 0 ? 0 : index - 1;
}

}  // namespace race_ego_bridge

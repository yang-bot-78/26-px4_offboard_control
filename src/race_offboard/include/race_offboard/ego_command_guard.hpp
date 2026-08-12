#pragma once

#include <algorithm>
#include <cmath>

namespace race_offboard
{

inline double wrapEgoCommandAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool localPositionJumpIsPlausible(
  const double distance_m, const double dt_sec, const double maximum_speed_mps,
  const double fixed_allowance_m)
{
  return std::isfinite(distance_m) && std::isfinite(dt_sec) && dt_sec > 0.0 &&
         distance_m <= fixed_allowance_m + maximum_speed_mps * dt_sec;
}

inline bool egoSetpointIsNearTrustedPosition(
  const double horizontal_distance_m, const double maximum_lead_m)
{
  return std::isfinite(horizontal_distance_m) && horizontal_distance_m <= maximum_lead_m;
}

inline bool initialPositionSampleIsStable(
  const double distance_from_candidate_m, const double speed_mps,
  const double maximum_spread_m, const double maximum_speed_mps)
{
  return std::isfinite(distance_from_candidate_m) && std::isfinite(speed_mps) &&
         distance_from_candidate_m <= maximum_spread_m && speed_mps <= maximum_speed_mps;
}

inline double slewEgoYaw(
  const double previous_yaw, const double requested_yaw, const double max_rate_rad_s,
  const double dt_sec)
{
  const double max_step = std::max(0.0, max_rate_rad_s) * std::max(0.0, dt_sec);
  const double delta = wrapEgoCommandAngle(requested_yaw - previous_yaw);
  return wrapEgoCommandAngle(previous_yaw + std::clamp(delta, -max_step, max_step));
}

}  // namespace race_offboard

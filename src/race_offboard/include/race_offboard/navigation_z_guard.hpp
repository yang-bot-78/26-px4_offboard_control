#pragma once

#include <algorithm>
#include <cmath>

namespace race_offboard
{

struct NavigationZLimitResult
{
  double value{0.0};
  bool limited{false};
};

// Limits a local-NED position target to a bounded vertical slew. The first
// command establishes the reference; subsequent commands cannot turn a
// localization or planner-height outlier into an instantaneous climb/descent.
class NavigationZGuard
{
public:
  void configure(double maximum_step_m, double maximum_rate_mps)
  {
    maximum_step_m_ = std::max(0.001, maximum_step_m);
    maximum_rate_mps_ = std::max(0.001, maximum_rate_mps);
  }

  void reset()
  {
    have_previous_ = false;
  }

  NavigationZLimitResult limit(double requested_z_ned, double now_s)
  {
    if (!std::isfinite(requested_z_ned) || !std::isfinite(now_s)) {
      return {requested_z_ned, false};
    }
    if (!have_previous_) {
      have_previous_ = true;
      previous_z_ned_ = requested_z_ned;
      previous_time_s_ = now_s;
      return {requested_z_ned, false};
    }

    const double elapsed_s = std::max(0.0, now_s - previous_time_s_);
    const double allowed_delta_m = std::min(
      maximum_step_m_, maximum_rate_mps_ * elapsed_s);
    const double limited_z_ned = std::clamp(
      requested_z_ned, previous_z_ned_ - allowed_delta_m,
      previous_z_ned_ + allowed_delta_m);
    const bool limited = std::abs(limited_z_ned - requested_z_ned) > 1.0e-9;
    previous_z_ned_ = limited_z_ned;
    previous_time_s_ = now_s;
    return {limited_z_ned, limited};
  }

private:
  double maximum_step_m_{0.08};
  double maximum_rate_mps_{0.35};
  double previous_z_ned_{0.0};
  double previous_time_s_{0.0};
  bool have_previous_{false};
};

}  // namespace race_offboard

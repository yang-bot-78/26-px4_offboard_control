#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace race_offboard
{

enum class Px4LocalPoseResetDecision
{
  NoPending,
  Matched,
  Mismatched,
  Expired,
};

struct Px4LocalPoseResetObservation
{
  Px4LocalPoseResetDecision decision{Px4LocalPoseResetDecision::NoPending};
  std::array<double, 3> expected_delta_ned{0.0, 0.0, 0.0};
  std::array<double, 3> observed_delta_ned{0.0, 0.0, 0.0};
  double xy_residual_m{0.0};
  double z_residual_m{0.0};
  bool full_xy_metadata{false};
};

// A reset is a one-frame authorization, not a relaxation of the normal jump
// guard. The next local-pose delta must also agree in XY.
class Px4LocalPoseResetMatcher
{
public:
  void configure(double window_sec, double xy_tolerance_m, double z_tolerance_m)
  {
    window_ns_ = static_cast<std::int64_t>(std::max(0.05, window_sec) * 1.0e9);
    xy_tolerance_m_ = std::max(0.01, xy_tolerance_m);
    z_tolerance_m_ = std::max(0.01, z_tolerance_m);
  }

  void registerReset(
    const std::array<double, 3> & delta_ned, bool full_xy_metadata, std::int64_t now_ns)
  {
    if (!finite(delta_ned)) {
      return;
    }
    expected_delta_ned_ = delta_ned;
    full_xy_metadata_ = full_xy_metadata;
    registered_at_ns_ = now_ns;
    pending_ = true;
  }

  bool pending() const
  {
    return pending_;
  }

  Px4LocalPoseResetObservation observe(
    const std::array<double, 3> & observed_delta_ned, std::int64_t now_ns)
  {
    Px4LocalPoseResetObservation observation;
    if (!pending_) {
      return observation;
    }
    observation.expected_delta_ned = expected_delta_ned_;
    observation.observed_delta_ned = observed_delta_ned;
    observation.full_xy_metadata = full_xy_metadata_;
    if (now_ns < registered_at_ns_ || now_ns - registered_at_ns_ > window_ns_) {
      pending_ = false;
      observation.decision = Px4LocalPoseResetDecision::Expired;
      return observation;
    }

    pending_ = false;
    observation.xy_residual_m = std::hypot(
      observed_delta_ned[0] - expected_delta_ned_[0],
      observed_delta_ned[1] - expected_delta_ned_[1]);
    observation.z_residual_m = std::fabs(observed_delta_ned[2] - expected_delta_ned_[2]);
    observation.decision = observation.xy_residual_m <= xy_tolerance_m_ &&
      observation.z_residual_m <= z_tolerance_m_ ?
      Px4LocalPoseResetDecision::Matched : Px4LocalPoseResetDecision::Mismatched;
    return observation;
  }

private:
  static bool finite(const std::array<double, 3> & value)
  {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
  }

  std::array<double, 3> expected_delta_ned_{0.0, 0.0, 0.0};
  std::int64_t registered_at_ns_{0};
  std::int64_t window_ns_{350000000};
  double xy_tolerance_m_{0.10};
  double z_tolerance_m_{0.10};
  bool full_xy_metadata_{false};
  bool pending_{false};
};

}  // namespace race_offboard

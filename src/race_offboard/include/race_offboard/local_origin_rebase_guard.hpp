#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace race_offboard
{

inline bool initialGroundReferenceLockAllowed(
  bool armed, bool require_ev_health, bool ev_ready)
{
  return !armed && (!require_ev_health || ev_ready);
}

inline bool localOriginRebaseAllowed(
  bool armed, bool vehicle_idle, bool control_idle_hold, bool ev_ready,
  bool hold_position_valid)
{
  return !armed && vehicle_idle && control_idle_hold && ev_ready && hold_position_valid;
}

enum class LocalOriginRebaseDecision
{
  None,
  CandidateStarted,
  CandidateReset,
  Confirmed
};

// Confirms that a discontinuous local-position frame has settled before a
// disarmed IDLE controller is allowed to replace its old hold reference.
class LocalOriginRebaseGuard
{
public:
  void configure(double stabilization_s, double max_spread_m, double max_speed_mps)
  {
    stabilization_s_ = std::max(0.1, stabilization_s);
    max_spread_m_ = std::max(0.01, max_spread_m);
    max_speed_mps_ = std::max(0.01, max_speed_mps);
  }

  void reset()
  {
    active_ = false;
    stable_since_ns_ = 0;
    candidate_ = {0.0, 0.0, 0.0};
  }

  bool active() const {return active_;}

  LocalOriginRebaseDecision observe(
    const std::array<double, 3> & position, double speed_mps, std::int64_t now_ns)
  {
    if (!finite(position) || !std::isfinite(speed_mps) || speed_mps > max_speed_mps_) {
      reset();
      return LocalOriginRebaseDecision::None;
    }

    if (!active_) {
      beginCandidate(position, now_ns);
      return LocalOriginRebaseDecision::CandidateStarted;
    }

    if (now_ns < stable_since_ns_ || distance(position, candidate_) > max_spread_m_) {
      beginCandidate(position, now_ns);
      return LocalOriginRebaseDecision::CandidateReset;
    }

    const double stable_s = static_cast<double>(now_ns - stable_since_ns_) / 1.0e9;
    if (stable_s >= stabilization_s_) {
      reset();
      return LocalOriginRebaseDecision::Confirmed;
    }
    return LocalOriginRebaseDecision::None;
  }

private:
  static bool finite(const std::array<double, 3> & value)
  {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
  }

  static double distance(
    const std::array<double, 3> & lhs, const std::array<double, 3> & rhs)
  {
    return std::hypot(
      std::hypot(lhs[0] - rhs[0], lhs[1] - rhs[1]), lhs[2] - rhs[2]);
  }

  void beginCandidate(const std::array<double, 3> & position, std::int64_t now_ns)
  {
    candidate_ = position;
    stable_since_ns_ = now_ns;
    active_ = true;
  }

  double stabilization_s_{1.0};
  double max_spread_m_{0.08};
  double max_speed_mps_{0.20};
  std::array<double, 3> candidate_{0.0, 0.0, 0.0};
  std::int64_t stable_since_ns_{0};
  bool active_{false};
};

}  // namespace race_offboard

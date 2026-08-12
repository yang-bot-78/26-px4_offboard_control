#pragma once

#include <algorithm>
#include <cstdint>
#include <string>

// External-vision health gate, ported from EvHealthTracker in
// src/px4_ros_com/scripts/minipc_mavros_offboard.py.
//
// The simulation Offboard node had no EV health check at all: PX4 SITL always
// has a valid estimate.  On the real aircraft the EKF2 position comes from
// FAST-LIO through /mavros/vision_pose/pose_cov, and that chain can degrade
// while MAVROS still reports a healthy link.  Arming on a degrading EV estimate
// is the failure mode this guards.
//
// Kept free of ROS types deliberately -- time enters as a nanosecond count so
// the state machine is unit-testable without a node.
namespace race_offboard
{

enum class EvHealthState
{
  Unknown,
  Healthy,
  Suspect,
  Fault
};

inline const char * evHealthStateName(EvHealthState state)
{
  switch (state) {
    case EvHealthState::Healthy: return "HEALTHY";
    case EvHealthState::Suspect: return "SUSPECT";
    case EvHealthState::Fault: return "FAULT";
    default: return "UNKNOWN";
  }
}

// Unrecognised text is treated as FAULT, never as "probably fine".
inline EvHealthState parseEvHealthState(const std::string & text, bool * recognised = nullptr)
{
  std::string normalized;
  normalized.reserve(text.size());
  for (const char c : text) {
    if (!std::isspace(static_cast<unsigned char>(c))) {
      normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
  }
  if (recognised != nullptr) {
    *recognised = true;
  }
  if (normalized == "HEALTHY") {return EvHealthState::Healthy;}
  if (normalized == "SUSPECT") {return EvHealthState::Suspect;}
  if (normalized == "FAULT") {return EvHealthState::Fault;}
  if (normalized == "UNKNOWN") {return EvHealthState::Unknown;}
  if (recognised != nullptr) {
    *recognised = false;
  }
  return EvHealthState::Fault;
}

class EvHealthTracker
{
public:
  EvHealthTracker() = default;

  void configure(double required_s, double freshness_s)
  {
    required_s_ = std::max(0.0, required_s);
    freshness_s_ = std::max(0.1, freshness_s);
  }

  // Returns true when the state changed or the HEALTHY continuity restarted,
  // which the caller uses to decide whether to log.
  bool update(const std::string & status_text, std::int64_t now_ns)
  {
    bool recognised = true;
    const EvHealthState new_state = parseEvHealthState(status_text, &recognised);
    const EvHealthState previous_state = state_;
    const bool had_status = have_status_;
    const std::int64_t previous_status_ns = last_status_ns_;
    bool continuity_restarted = false;

    if (!recognised) {
      last_reason_ = "invalid EV health status '" + status_text + "'";
    }

    if (new_state == EvHealthState::Healthy) {
      // A gap longer than the freshness window means the stream was
      // interrupted, so the continuous-healthy timer must restart rather than
      // credit the operator for time when nothing was being reported.
      const double gap_s = had_status ?
        static_cast<double>(now_ns - previous_status_ns) / 1.0e9 : -1.0;
      if (previous_state != EvHealthState::Healthy || !had_status || gap_s < 0.0 ||
        gap_s > freshness_s_)
      {
        healthy_since_ns_ = now_ns;
        have_healthy_since_ = true;
        continuity_restarted = true;
      }
      if (recognised) {
        last_reason_.clear();
      }
    } else {
      have_healthy_since_ = false;
      if (recognised) {
        last_reason_ = std::string("EV health state is ") + evHealthStateName(new_state);
      }
    }

    state_ = new_state;
    last_status_ns_ = now_ns;
    have_status_ = true;
    return state_ != previous_state || continuity_restarted;
  }

  bool isFresh(std::int64_t now_ns) const
  {
    if (!have_status_) {
      return false;
    }
    const double age_s = static_cast<double>(now_ns - last_status_ns_) / 1.0e9;
    return age_s >= 0.0 && age_s <= freshness_s_;
  }

  double healthyDurationS(std::int64_t now_ns) const
  {
    if (!have_healthy_since_ || state_ != EvHealthState::Healthy) {
      return 0.0;
    }
    return std::max(0.0, static_cast<double>(now_ns - healthy_since_ns_) / 1.0e9);
  }

  // Pre-arm gate: HEALTHY, fresh, and continuously so for required_s.
  bool ready(std::int64_t now_ns) const
  {
    return state_ == EvHealthState::Healthy && isFresh(now_ns) &&
           healthyDurationS(now_ns) >= required_s_;
  }

  // In-flight gate.  Only FAULT triggers a landing: SUSPECT and staleness are
  // reported but do not by themselves bring the aircraft down, matching the
  // behaviour already flown by minipc_mavros_offboard.py.
  bool faulted() const
  {
    return state_ == EvHealthState::Fault;
  }

  bool unsafeReason(std::int64_t now_ns, std::string * reason) const
  {
    if (!have_status_) {
      if (reason != nullptr) {*reason = "EV health status has not been received";}
      return true;
    }
    const double age_s = static_cast<double>(now_ns - last_status_ns_) / 1.0e9;
    if (age_s < 0.0) {
      if (reason != nullptr) {*reason = "EV health status timestamp is in the future";}
      return true;
    }
    if (age_s > freshness_s_) {
      if (reason != nullptr) {
        *reason = "EV health status timeout: age=" + std::to_string(age_s) +
          "s > freshness=" + std::to_string(freshness_s_) + "s";
      }
      return true;
    }
    if (state_ != EvHealthState::Healthy) {
      if (reason != nullptr) {
        *reason = last_reason_.empty() ?
          std::string("EV health state is ") + evHealthStateName(state_) : last_reason_;
      }
      return true;
    }
    return false;
  }

  EvHealthState state() const {return state_;}
  double requiredS() const {return required_s_;}

private:
  double required_s_{7.5};
  double freshness_s_{1.0};
  EvHealthState state_{EvHealthState::Unknown};
  std::int64_t last_status_ns_{0};
  std::int64_t healthy_since_ns_{0};
  bool have_status_{false};
  bool have_healthy_since_{false};
  std::string last_reason_{"no EV health status received"};
};

}  // namespace race_offboard

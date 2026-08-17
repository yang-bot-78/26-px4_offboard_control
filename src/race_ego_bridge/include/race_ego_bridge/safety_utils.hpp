#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
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

inline double trajectorySwitchEvaluationTime(
  double elapsed, double duration, bool measured_position_hold)
{
  return measured_position_hold ? 0.0 : clampTrajectoryTime(elapsed, duration);
}

inline bool replanHoldReadyToReanchor(
  double horizontal_speed, double stable_age_sec,
  double maximum_horizontal_speed, double required_stable_sec)
{
  return std::isfinite(horizontal_speed) && std::isfinite(stable_age_sec) &&
    std::isfinite(maximum_horizontal_speed) && std::isfinite(required_stable_sec) &&
    horizontal_speed <= maximum_horizontal_speed && stable_age_sec >= required_stable_sec;
}

inline bool candidateStartsAfterReplanReady(
  double candidate_start_sec, double replan_ready_sec)
{
  return std::isfinite(candidate_start_sec) && std::isfinite(replan_ready_sec) &&
    candidate_start_sec >= replan_ready_sec;
}

inline bool shouldResetFlightHandover(bool was_armed, bool armed)
{
  return was_armed && !armed;
}

inline bool shouldPreserveTrajectoryForPlannerPause(
  bool have_trajectory, bool trajectory_collision_free, bool collision_latched)
{
  return have_trajectory && trajectory_collision_free && !collision_latched;
}

// A localization reset invalidates the coordinate frame used to generate the
// active trajectory.  Unlike a short planner-compute pause, it must never be
// resumed from that trajectory or from its local-goal transaction.
inline bool frlioRecoveryNeedsFreshLocalGoal(
  uint64_t received_local_goal_seq, uint64_t recovery_watermark_seq)
{
  return received_local_goal_seq <= recovery_watermark_seq;
}

inline bool transitionContinuous(const Vec3 & previous, const Vec3 & next, double max_jump)
{
  if (!finite(previous) || !finite(next) || !std::isfinite(max_jump) || max_jump < 0.0) {
    return false;
  }
  return std::hypot(
    std::hypot(next.x - previous.x, next.y - previous.y), next.z - previous.z) <= max_jump;
}

struct TrajectoryState
{
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
};

struct TrajectoryTransitionCheck
{
  bool continuous{false};
  double position_error{std::numeric_limits<double>::infinity()};
  double velocity_error{std::numeric_limits<double>::infinity()};
  double acceleration_error{std::numeric_limits<double>::infinity()};
};

inline TrajectoryTransitionCheck transitionStateContinuous(
  const TrajectoryState & previous, const TrajectoryState & next,
  double position_tolerance, double velocity_tolerance,
  double acceleration_tolerance)
{
  TrajectoryTransitionCheck result;
  if (!finite(previous.position) || !finite(previous.velocity) ||
    !finite(previous.acceleration) || !finite(next.position) ||
    !finite(next.velocity) || !finite(next.acceleration) ||
    !std::isfinite(position_tolerance) || position_tolerance < 0.0 ||
    !std::isfinite(velocity_tolerance) || velocity_tolerance < 0.0 ||
    !std::isfinite(acceleration_tolerance) || acceleration_tolerance < 0.0)
  {
    return result;
  }
  const auto norm = [](const Vec3 & lhs, const Vec3 & rhs) {
      return std::hypot(
        std::hypot(lhs.x - rhs.x, lhs.y - rhs.y), lhs.z - rhs.z);
    };
  result.position_error = norm(previous.position, next.position);
  result.velocity_error = norm(previous.velocity, next.velocity);
  result.acceleration_error = norm(previous.acceleration, next.acceleration);
  result.continuous = result.position_error <= position_tolerance &&
    result.velocity_error <= velocity_tolerance &&
    result.acceleration_error <= acceleration_tolerance;
  return result;
}

// Two distinct errors that a single "new trajectory start vs current odom"
// comparison used to conflate:
//
//   splice_error   = candidate(t_switch) - active_trajectory(t_switch)
//                    Purely trajectory-vs-trajectory. Judges handoff
//                    continuity, so it is the only one allowed to reject a
//                    candidate trajectory.
//   tracking_error = measured_state(now) - active_trajectory(now)
//                    Judges how well the vehicle (and therefore localization
//                    and control) is following the trajectory it already has.
//                    A large value here is a localization/tracking problem and
//                    must not be reported as a trajectory handoff failure.
//
// Keeping them separate is what lets one fault be diagnosed instead of two
// unrelated subsystems being blamed for the same number.
enum class TrajectoryFaultClass
{
  None,
  TrajectoryHandoff,     // splice large, tracking small
  LocalizationTracking,  // splice small, tracking large
  Both,                  // both large: report both, do not guess
};

inline const char * trajectoryFaultClassName(TrajectoryFaultClass value)
{
  switch (value) {
    case TrajectoryFaultClass::None: return "NONE";
    case TrajectoryFaultClass::TrajectoryHandoff: return "TRAJECTORY_HANDOFF";
    case TrajectoryFaultClass::LocalizationTracking: return "LOCALIZATION_TRACKING";
    case TrajectoryFaultClass::Both: return "TRAJECTORY_HANDOFF_AND_TRACKING";
  }
  return "NONE";
}

inline TrajectoryFaultClass classifyTrajectoryFault(
  double splice_error, double splice_tolerance,
  double tracking_error, double tracking_tolerance)
{
  // The splice error gates a real decision (reject the candidate), so a
  // non-finite value fails closed: we cannot show it is within tolerance.
  const bool splice_bad = !std::isfinite(splice_error) || splice_error > splice_tolerance;
  // The tracking error is diagnostic only. An unknown value means "there is no
  // command to compare against yet", which is not evidence of a fault, and
  // reporting one would produce spurious warnings before the first validated
  // output. Only a finite, over-threshold value counts.
  const bool tracking_bad = std::isfinite(tracking_tolerance) &&
    std::isfinite(tracking_error) && tracking_error > tracking_tolerance;
  if (splice_bad && tracking_bad) {
    return TrajectoryFaultClass::Both;
  }
  if (splice_bad) {
    return TrajectoryFaultClass::TrajectoryHandoff;
  }
  if (tracking_bad) {
    return TrajectoryFaultClass::LocalizationTracking;
  }
  return TrajectoryFaultClass::None;
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

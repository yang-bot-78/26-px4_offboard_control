#pragma once

#include <cmath>
#include <string>

namespace race_offboard
{

inline bool shouldTrackManualHandoverPosition(
  bool manual_handover, bool vehicle_idle, bool armed, bool offboard,
  bool have_finite_position)
{
  return manual_handover && vehicle_idle && armed && !offboard && have_finite_position;
}

inline bool canAcceptManualHandover(
  bool manual_handover, bool vehicle_idle, bool armed, bool offboard,
  bool have_finite_position, bool ev_ready, bool hold_aligned,
  bool speed_safe, bool height_safe)
{
  return manual_handover && vehicle_idle && armed && offboard &&
         have_finite_position && ev_ready && hold_aligned && speed_safe && height_safe;
}

// Once an automated takeoff/flight transaction owns OFFBOARD, an operator
// mode switch or disarm is an explicit revocation of that authority.  The
// node must latch the revocation for the remainder of the process instead of
// racing the RC by requesting OFFBOARD/arm again.
inline bool pilotOverrideRequested(
  bool automation_active, bool was_offboard, bool offboard,
  bool was_armed, bool armed)
{
  return automation_active &&
         ((was_offboard && !offboard) || (was_armed && !armed));
}

// Flat-flight safety holds preserve the configured flight level.  Capturing
// the measured altitude after every planner stop creates a downward ratchet
// and can incorrectly re-enter the takeoff gates.
inline double fixedAltitudeHoldZ(double cruise_z)
{
  return std::isfinite(cruise_z) ? cruise_z : 0.0;
}

// Keep the takeoff and EGO-handover gates separate.  A valid user goal may
// start a vertical takeoff without an EGO trajectory; horizontal tracking may
// not begin until that trajectory is fresh and has not been safety-rejected.
inline bool canStartIndependentTakeoff(const bool terminal_state)
{
  return !terminal_state;
}

inline bool canHandoverToEgo(
  const bool takeoff_height_reached,
  const bool fresh_ego_setpoint,
  const bool planner_failed)
{
  return takeoff_height_reached && fresh_ego_setpoint && !planner_failed;
}

// An EV fault may request AUTO.LAND only after the vertical takeoff has
// completed. Takeoff velocity and the short handover transient are expected
// to be non-zero; keep the pilot in charge during that phase instead.
inline bool shouldAutoLandForEvFault(
  const bool auto_land_enabled,
  const bool armed,
  const bool takeoff_complete,
  const bool already_landing)
{
  return auto_land_enabled && armed && takeoff_complete && !already_landing;
}

// A new final goal invalidates the previous EGO command before the planner can
// publish the first command for the new transaction.  This remains a planning
// wait until an explicit planner failure is reported.  It must not be folded
// into the in-flight command timeout: global planning time varies with route
// length and a late, validated first command is still safe to accept.
inline bool shouldHoldForInitialEgoTrajectory(
  const bool awaiting_initial_trajectory,
  const bool planner_reported_failure)
{
  return awaiting_initial_trajectory && !planner_reported_failure;
}

// The former initial wait timeout is retained only as an operator diagnostic
// threshold.  Exceeding it does not turn a pending route into a failure.
inline bool initialEgoTrajectoryWaitIsSlow(
  const double elapsed_sec,
  const double diagnostic_threshold_sec)
{
  return elapsed_sec >= 0.0 && diagnostic_threshold_sec > 0.0 &&
         elapsed_sec >= diagnostic_threshold_sec;
}

// EGO setpoints arrive at the controller rate after Bridge validation.  They
// must never be interpreted as repeated takeoff requests.
inline bool shouldRestartTakeoffForEgoSetpoint()
{
  return false;
}

// Strict PX4 estimator flags are opt-in.  SITL may validly report an
// unavailable heading reference before takeoff while still providing a finite
// local position suitable for the configured simulation control path.
inline bool shouldRequireStrictLocalPositionHealth(const bool requested)
{
  return requested;
}

// A distinct operator final goal begins a new navigation transaction.  It may
// clear a latch from the prior goal only after discarding the old setpoint;
// motion remains gated by a newly received fresh, Bridge-validated trajectory.
inline bool canResetPlannerLatchForNewFinalGoal(
  const bool distinct_goal, const bool terminal_state)
{
  return distinct_goal && !terminal_state;
}

// A bridge collision report rejects the trajectory currently under review.
// Brake while waiting for a newly validated replacement, but do not discard
// that replacement by turning this recoverable report into a permanent latch.
inline bool egoPlannerStatusRequiresPermanentLatch(const std::string & status)
{
  return status.find("EGO_OCCUPANCY_STALE") != std::string::npos ||
         status.find("EGO_REPLAN_REANCHOR_FAILED") != std::string::npos ||
         status.find("EMERGENCY_HOLD") != std::string::npos;
}

}  // namespace race_offboard

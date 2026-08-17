#pragma once

#include <cmath>
#include <string>

// Maps FR-LIO's high-rate localization status onto the two decisions this node
// actually has to make, kept strictly separate:
//
//   ev_usable      -- may PX4 keep fusing the external-vision stream?
//   planner_usable -- may the local planner keep switching trajectories?
//
// The distinction is the whole point.  A late LiDAR posterior (FAULT_STALE_LIDAR)
// means the pose is drifting on IMU propagation alone: not good enough to plan
// against, but still a real, continuous, correctly timestamped state that EKF2
// should keep fusing with an inflated covariance.  Only a status asserting the
// propagated state itself is unusable may take EV away.
//
// Free of ROS types on purpose so the mapping is unit-testable without a node.
namespace race_offboard
{

// FR-LIO status strings, in increasing severity.
//   HEALTHY                GOOD              plan + fuse
//   SUSPECT_STALE_LIDAR    DEGRADED          plan + fuse (inflated covariance)
//   FAULT_STALE_LIDAR      PLANNER_UNUSABLE  fuse only
//   FAULT_STATE_UNUSABLE   STATE_UNUSABLE    neither
//   WAITING_FOR_LIDAR / UNIT_UNCONFIRMED / DISABLED -- no usable state yet
inline bool frlioStatusAllowsPlanning(const std::string & status)
{
  return status == "HEALTHY" || status == "SUSPECT_STALE_LIDAR";
}

// Unrecognised text fails closed for planning but is NOT treated as an EV
// fault: an unknown status is a reason to stop planning, never a reason to
// interrupt the PX4 EV stream, because interrupting it is strictly more
// dangerous than continuing to fuse a covariance-inflated pose.
inline bool frlioStatusAllowsEvFusion(const std::string & status)
{
  return status != "FAULT_STATE_UNUSABLE" && status != "UNIT_UNCONFIRMED" &&
         status != "WAITING_FOR_LIDAR" && status != "DISABLED";
}

// A planner hold caused by LiDAR staleness is recoverable, so it must not
// cancel the goal.  Releasing it requires FR-LIO to report a usable planner
// state AND the vehicle to have been stationary for `required_stable_sec`:
// resuming while still braking would hand the planner a moving start state.
inline bool frlioHoldReleaseReady(
  const bool hold_active, const bool planner_usable, const double stable_sec,
  const double required_stable_sec, const double horizontal_speed_mps,
  const double speed_tolerance_mps)
{
  return hold_active && planner_usable && std::isfinite(stable_sec) &&
         std::isfinite(horizontal_speed_mps) && stable_sec >= required_stable_sec &&
         horizontal_speed_mps <= speed_tolerance_mps;
}

// Planner HOLD must never stop the Offboard setpoint stream.  PX4 drops out of
// OFFBOARD after ~0.5 s without a setpoint, so a planner-side hold that stopped
// publishing would convert a localization degradation into a mode loss.  Every
// hold state below is required to keep publishing a valid hold setpoint.
inline bool holdStateMustKeepPublishingSetpoints(const bool armed, const bool offboard_active)
{
  return armed || offboard_active;
}

}  // namespace race_offboard

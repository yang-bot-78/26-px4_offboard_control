#pragma once

#include <array>
#include <cmath>
#include <cstdint>

// MAVROS boundary conversions for the Offboard node.
//
// The node keeps every internal state variable and every planner contract in
// NED, exactly as the PX4 DDS version did.  Only the MAVROS boundary converts:
//
//   inbound   /mavros/local_position/pose + velocity_local  (ENU) -> NED
//   outbound  mavros_msgs/PositionTarget                     NED  -> (ENU)
//
// The planner's unified map is standard ROS ENU. The ENU<->NED pair below is
// therefore the ordinary axis swap at the MAVROS/PX4 boundary.
//
// IMPORTANT: /mavros/setpoint_raw/local expects ENU input even though its
// coordinate_frame constant is named FRAME_LOCAL_NED; the MAVROS plugin does
// the ENU->NED conversion internally before emitting MAVLink.  Publishing raw
// NED numbers there silently swaps North and East and flies the wrong way.
namespace race_offboard
{

inline double wrapFrameAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Standard ENU (East, North, Up) -> NED (North, East, Down).
inline std::array<double, 3> enuToNed(const std::array<double, 3> & enu)
{
  return {enu[1], enu[0], -enu[2]};
}

// Standard NED -> ENU.  Self-inverse in the same way as enuToNed.
inline std::array<double, 3> nedToEnu(const std::array<double, 3> & ned)
{
  return {ned[1], ned[0], -ned[2]};
}

// ENU yaw is measured from East toward North; NED yaw from North toward East.
inline double enuYawToNed(double yaw_enu)
{
  return wrapFrameAngle(M_PI_2 - yaw_enu);
}

inline double nedYawToEnu(double yaw_ned)
{
  return wrapFrameAngle(M_PI_2 - yaw_ned);
}

// Planar rigid transform from the global map ENU frame to PX4 local ENU.
struct PlanarFrameTransform
{
  double yaw{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline std::array<double, 3> mapToLocalPosition(
  const std::array<double, 3> & map, const PlanarFrameTransform & transform)
{
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  return {
    c * map[0] - s * map[1] + transform.x,
    s * map[0] + c * map[1] + transform.y,
    map[2] + transform.z};
}

inline std::array<double, 3> mapToLocalVector(
  const std::array<double, 3> & map, const PlanarFrameTransform & transform)
{
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  return {c * map[0] - s * map[1], s * map[0] + c * map[1], map[2]};
}

inline double mapToLocalYaw(double map_yaw, const PlanarFrameTransform & transform)
{
  return wrapFrameAngle(map_yaw + transform.yaw);
}

// PX4's TrajectorySetpoint marks an ignored axis with NaN.  MAVROS instead uses
// a type_mask bitfield, so the per-axis NaN pattern has to be translated rather
// than mapped to one fixed constant: the over-height soft guard NaNs only the Z
// feed-forward and must keep XY velocity/acceleration live.
struct PositionTargetMaskBits
{
  static constexpr std::uint16_t kIgnoreVx = 8;
  static constexpr std::uint16_t kIgnoreVy = 16;
  static constexpr std::uint16_t kIgnoreVz = 32;
  static constexpr std::uint16_t kIgnoreAfx = 64;
  static constexpr std::uint16_t kIgnoreAfy = 128;
  static constexpr std::uint16_t kIgnoreAfz = 256;
  static constexpr std::uint16_t kIgnoreYaw = 1024;
  static constexpr std::uint16_t kIgnoreYawRate = 2048;
};

// Build a type_mask from the finiteness of each feed-forward axis.
//
// velocity_enu / acceleration_enu are already converted to ENU, so index 0 is
// East and index 1 is North.  A NaN on an axis sets that axis's ignore bit.
inline std::uint16_t positionTargetTypeMask(
  const std::array<double, 3> & velocity_enu,
  const std::array<double, 3> & acceleration_enu,
  bool yaw_rate_valid)
{
  std::uint16_t mask = 0;
  if (!std::isfinite(velocity_enu[0])) {mask |= PositionTargetMaskBits::kIgnoreVx;}
  if (!std::isfinite(velocity_enu[1])) {mask |= PositionTargetMaskBits::kIgnoreVy;}
  if (!std::isfinite(velocity_enu[2])) {mask |= PositionTargetMaskBits::kIgnoreVz;}
  if (!std::isfinite(acceleration_enu[0])) {mask |= PositionTargetMaskBits::kIgnoreAfx;}
  if (!std::isfinite(acceleration_enu[1])) {mask |= PositionTargetMaskBits::kIgnoreAfy;}
  if (!std::isfinite(acceleration_enu[2])) {mask |= PositionTargetMaskBits::kIgnoreAfz;}
  if (!yaw_rate_valid) {mask |= PositionTargetMaskBits::kIgnoreYawRate;}
  return mask;
}

// Zero out an axis that the mask ignores.  MAVROS forwards whatever sits in the
// field regardless of the mask on some paths, and NaN in a MAVLink float is
// worse than a masked-off zero.
inline std::array<double, 3> sanitizeIgnoredAxes(const std::array<double, 3> & value)
{
  return {
    std::isfinite(value[0]) ? value[0] : 0.0,
    std::isfinite(value[1]) ? value[1] : 0.0,
    std::isfinite(value[2]) ? value[2] : 0.0};
}

}  // namespace race_offboard

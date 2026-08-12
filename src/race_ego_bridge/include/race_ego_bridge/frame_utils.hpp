#pragma once

#include <algorithm>
#include <cmath>

namespace race_ego_bridge
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TimedYawPose
{
  double stamp_sec{0.0};
  Vec3 position;
  double yaw{0.0};
};

inline Vec3 nedToMap(const Vec3 & ned)
{
  return Vec3{ned.y, ned.x, -ned.z};
}

inline Vec3 mapToNed(const Vec3 & map)
{
  return Vec3{map.y, map.x, -map.z};
}

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double nedYawToMap(double yaw_ned)
{
  return wrapAngle(M_PI_2 - yaw_ned);
}

inline double mapYawToNed(double yaw_map)
{
  return wrapAngle(M_PI_2 - yaw_map);
}

// The unified project map is standard ROS ENU. World alignment happens before
// data enters map, so the EGO bridge must not add another fixed yaw here.
inline Vec3 enuToMap(const Vec3 & enu)
{
  return enu;
}

inline Vec3 mapToEnu(const Vec3 & map)
{
  return map;
}

inline double enuYawToMap(double yaw_enu)
{
  return wrapAngle(yaw_enu);
}

inline double mapYawToEnu(double yaw_map)
{
  return wrapAngle(yaw_map);
}

inline TimedYawPose interpolateYawPose(
  const TimedYawPose & first, const TimedYawPose & second, double stamp_sec)
{
  if (!std::isfinite(stamp_sec) || second.stamp_sec <= first.stamp_sec) {
    return first;
  }
  const double ratio = std::clamp(
    (stamp_sec - first.stamp_sec) / (second.stamp_sec - first.stamp_sec), 0.0, 1.0);
  const double yaw_delta = wrapAngle(second.yaw - first.yaw);
  return TimedYawPose{
    stamp_sec,
    Vec3{
      first.position.x + ratio * (second.position.x - first.position.x),
      first.position.y + ratio * (second.position.y - first.position.y),
      first.position.z + ratio * (second.position.z - first.position.z)},
    wrapAngle(first.yaw + ratio * yaw_delta)};
}

inline bool finite(const Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Project a body-frame lidar point into the map ENU frame using the vehicle
// pose that PX4 reports, instead of relying on the SLAM pose.
//
// Why this exists: FAST-LIO's /cloud_registered is already in camera_init, but
// it is registered with FAST-LIO's own attitude estimate. When that estimate is
// wrong the whole cloud rotates with it, so no downstream transform can repair
// it. The body-frame cloud (/cloud_registered_body, equivalently
// /livox/lidar shifted by the IMU-lidar offset) carries no SLAM attitude at
// all, so projecting it with the PX4 pose keeps the map and the EGO odometry on
// one single pose source.
//
// body_point is expressed in the lidar/IMU body frame (x forward, y left,
// z up). map_position and yaw_map are the vehicle pose already converted to
// map ENU. Only yaw is applied: the flight profile
// is level cruise, and PX4 roll/pitch during it stay within a few degrees, so a
// yaw-only projection avoids feeding attitude noise into the occupancy map.
inline Vec3 bodyPointToMap(
  const Vec3 & body_point, const Vec3 & map_position, double yaw_map)
{
  const double cos_yaw = std::cos(yaw_map);
  const double sin_yaw = std::sin(yaw_map);
  return Vec3{
    map_position.x + body_point.x * cos_yaw - body_point.y * sin_yaw,
    map_position.y + body_point.x * sin_yaw + body_point.y * cos_yaw,
    map_position.z + body_point.z};
}

inline bool insideGeofence(
  const Vec3 & map, double min_x, double max_x, double min_y, double max_y,
  double min_height, double max_height)
{
  return finite(map) && map.x >= min_x && map.x <= max_x && map.y >= min_y &&
         map.y <= max_y && map.z >= min_height && map.z <= max_height;
}

}  // namespace race_ego_bridge

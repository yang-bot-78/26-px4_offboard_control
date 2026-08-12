#ifndef EGO_PLANNER_YAW_TIMING_POLICY_H_
#define EGO_PLANNER_YAW_TIMING_POLICY_H_

#include <cmath>

namespace ego_planner
{

class YawTimingPolicy
{
public:
  // Gazebo can invoke a wall timer more than once at the same simulation time.
  // A yaw-rate derivative is only valid once ROS time has moved forward.
  static bool canCalculateYawRate(const double elapsed_sec)
  {
    return std::isfinite(elapsed_sec) && elapsed_sec > 1.0e-6;
  }
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_YAW_TIMING_POLICY_H_

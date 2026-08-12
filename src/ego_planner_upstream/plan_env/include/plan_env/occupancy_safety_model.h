#pragma once

#include <cmath>
#include <limits>

namespace plan_env
{

// GridMap stores occupancy per voxel, so the configured center-clearance is
// rounded outward to a whole number of voxels before being applied.
struct OccupancySafetyModel
{
  static int inflationSteps(const double required_center_clearance, const double resolution)
  {
    if (!std::isfinite(required_center_clearance) || !std::isfinite(resolution) ||
        required_center_clearance <= 0.0 || resolution <= 0.0)
    {
      return 0;
    }
    return static_cast<int>(std::ceil(required_center_clearance / resolution));
  }

  // The binary inflated grid is only a coarse representation. Rounding it
  // inward avoids turning a continuous 0.509 m contract into a 0.600 m hard
  // constraint; all safety-critical callers use the continuous query below.
  static int coarseInflationSteps(const double required_center_clearance, const double resolution)
  {
    if (!std::isfinite(required_center_clearance) || !std::isfinite(resolution) ||
        required_center_clearance <= 0.0 || resolution <= 0.0)
    {
      return 0;
    }
    return static_cast<int>(std::floor(required_center_clearance / resolution));
  }

  static double effectiveClearance(const double required_center_clearance, const double resolution)
  {
    const int steps = inflationSteps(required_center_clearance, resolution);
    return steps > 0 ? static_cast<double>(steps) * resolution
                     : std::numeric_limits<double>::quiet_NaN();
  }

  // Safety-critical trajectory checks compare a floating-point point-to-raw-
  // obstacle distance with the one centre-clearance contract, without first
  // rounding the sample to an inflated voxel.
  static bool isContinuouslySafe(
      const double measured_clearance, const double required_center_clearance)
  {
    return std::isfinite(measured_clearance) && std::isfinite(required_center_clearance) &&
           measured_clearance >= required_center_clearance;
  }
};

}  // namespace plan_env

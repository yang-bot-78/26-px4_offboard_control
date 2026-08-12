#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace race_super_planner_ros2::policy
{

struct SegmentPoint2D
{
  double x{0.0};
  double y{0.0};
};

inline std::size_t segmentSampleCount(
  const SegmentPoint2D & start, const SegmentPoint2D & end, double maximum_step)
{
  const double length = std::hypot(end.x - start.x, end.y - start.y);
  return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(length / maximum_step)));
}

inline SegmentPoint2D segmentSample(
  const SegmentPoint2D & start, const SegmentPoint2D & end, std::size_t sample_index,
  std::size_t sample_count)
{
  const double ratio = static_cast<double>(sample_index) / static_cast<double>(sample_count);
  return {
    start.x + (end.x - start.x) * ratio,
    start.y + (end.y - start.y) * ratio};
}

}  // namespace race_super_planner_ros2::policy

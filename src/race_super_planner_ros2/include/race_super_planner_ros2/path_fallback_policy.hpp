#pragma once

#include <cstddef>

namespace race_super_planner_ros2
{

enum class PathValidationFailure
{
  NONE,
  EMPTY,
  NON_FINITE,
  OUT_OF_BOUNDS,
  COLLISION,
  SMOOTHING_VALIDATION,
};

struct PathValidationResult
{
  PathValidationFailure failure{PathValidationFailure::NONE};
  std::size_t index{0};

  bool valid() const
  {
    return failure == PathValidationFailure::NONE;
  }
};

enum class PathSource
{
  NONE,
  SMOOTHED,
  RAW_ASTAR,
};

// This policy intentionally contains no tolerance or geometry mutation.  It
// only selects an already fully validated path, preferring the smooth result.
inline PathSource selectValidatedPath(
  const PathValidationResult & smoothed,
  const PathValidationResult & raw)
{
  if (smoothed.valid()) {
    return PathSource::SMOOTHED;
  }
  if (raw.valid()) {
    return PathSource::RAW_ASTAR;
  }
  return PathSource::NONE;
}

}  // namespace race_super_planner_ros2

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace race_super_planner_ros2::smoothing
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ClearanceAssessment
{
  bool finite{false};
  bool segments_safe{false};
  bool global_clearance_preserved{false};
  bool local_clearance_preserved{false};
  bool accepted{false};
  double baseline_minimum{std::numeric_limits<double>::infinity()};
  double candidate_minimum{std::numeric_limits<double>::infinity()};
  double maximum_progress_matched_loss{0.0};
};

struct SmoothingResult
{
  std::vector<Point3> path;
  ClearanceAssessment assessment;
  std::size_t accepted_updates{0};
  std::size_t accepted_iterations{0};
  bool changed{false};
};

inline bool finite(const Point3 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

inline double distance2d(const Point3 & first, const Point3 & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

inline Point3 interpolate(const Point3 & first, const Point3 & second, const double ratio)
{
  const double bounded = std::clamp(ratio, 0.0, 1.0);
  return {
    first.x + (second.x - first.x) * bounded,
    first.y + (second.y - first.y) * bounded,
    first.z + (second.z - first.z) * bounded};
}

inline double pathLength(const std::vector<Point3> & path)
{
  double length = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    length += distance2d(path[index - 1], path[index]);
  }
  return length;
}

inline Point3 sampleAtProgress(const std::vector<Point3> & path, const double progress)
{
  if (path.empty()) {
    return {};
  }
  if (path.size() == 1) {
    return path.front();
  }
  const double total = pathLength(path);
  if (total <= 1.0e-12) {
    return path.front();
  }
  const double target = std::clamp(progress, 0.0, 1.0) * total;
  double traversed = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const double segment = distance2d(path[index - 1], path[index]);
    if (traversed + segment >= target || index + 1 == path.size()) {
      const double ratio = segment <= 1.0e-12 ? 0.0 : (target - traversed) / segment;
      return interpolate(path[index - 1], path[index], ratio);
    }
    traversed += segment;
  }
  return path.back();
}

template<typename ClearanceFn>
double minimumClearance(
  const std::vector<Point3> & path, ClearanceFn clearance, const double maximum_step)
{
  double minimum = std::numeric_limits<double>::infinity();
  if (path.empty()) {
    return minimum;
  }
  minimum = clearance(path.front());
  for (std::size_t index = 1; index < path.size(); ++index) {
    const Point3 & start = path[index - 1];
    const Point3 & finish = path[index];
    const std::size_t samples = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(
        distance2d(start, finish) / std::max(1.0e-6, maximum_step))));
    for (std::size_t sample = 1; sample <= samples; ++sample) {
      minimum = std::min(
        minimum, clearance(
          interpolate(
            start, finish, static_cast<double>(sample) / static_cast<double>(samples))));
    }
  }
  return minimum;
}

inline bool clearanceNotWorse(
  const double candidate, const double baseline, const double numeric_tolerance)
{
  if (std::isinf(baseline) && std::isinf(candidate)) {
    return true;
  }
  if (!std::isfinite(candidate)) {
    return candidate > 0.0;
  }
  if (!std::isfinite(baseline)) {
    return false;
  }
  return candidate + numeric_tolerance >= baseline;
}

template<typename ClearanceFn, typename SegmentSafeFn>
ClearanceAssessment assess(
  const std::vector<Point3> & baseline,
  const std::vector<Point3> & candidate,
  ClearanceFn clearance,
  SegmentSafeFn segment_safe,
  const double maximum_step,
  const double numeric_tolerance)
{
  ClearanceAssessment result;
  result.finite = candidate.size() >= 2 &&
    std::all_of(candidate.begin(), candidate.end(), finite);
  if (!result.finite || baseline.size() < 2) {
    return result;
  }
  result.segments_safe = true;
  for (std::size_t index = 1; index < candidate.size(); ++index) {
    if (!segment_safe(candidate[index - 1], candidate[index])) {
      result.segments_safe = false;
      return result;
    }
  }
  result.baseline_minimum = minimumClearance(baseline, clearance, maximum_step);
  result.candidate_minimum = minimumClearance(candidate, clearance, maximum_step);
  result.global_clearance_preserved = clearanceNotWorse(
    result.candidate_minimum, result.baseline_minimum, numeric_tolerance);

  const std::size_t progress_samples = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(
      std::max(pathLength(baseline), pathLength(candidate)) /
      std::max(1.0e-6, maximum_step))));
  result.local_clearance_preserved = true;
  for (std::size_t index = 0; index <= progress_samples; ++index) {
    const double progress =
      static_cast<double>(index) / static_cast<double>(progress_samples);
    const double baseline_clearance = clearance(sampleAtProgress(baseline, progress));
    const double candidate_clearance = clearance(sampleAtProgress(candidate, progress));
    if (std::isfinite(baseline_clearance) && std::isfinite(candidate_clearance)) {
      result.maximum_progress_matched_loss = std::max(
        result.maximum_progress_matched_loss,
        baseline_clearance - candidate_clearance);
    }
    if (!clearanceNotWorse(
        candidate_clearance, baseline_clearance, numeric_tolerance))
    {
      result.local_clearance_preserved = false;
      break;
    }
  }
  result.accepted = result.segments_safe &&
    result.global_clearance_preserved && result.local_clearance_preserved;
  return result;
}

template<typename ClearanceFn, typename SegmentSafeFn>
SmoothingResult acceptedStepSmooth(
  const std::vector<Point3> & raw_baseline,
  const int iterations,
  ClearanceFn clearance,
  SegmentSafeFn segment_safe,
  const double maximum_step,
  const double numeric_tolerance)
{
  SmoothingResult result;
  result.path = raw_baseline;
  result.assessment = assess(
    raw_baseline, raw_baseline, clearance, segment_safe,
    maximum_step, numeric_tolerance);
  if (raw_baseline.size() < 3 || iterations <= 0 || !result.assessment.accepted) {
    return result;
  }

  constexpr std::array<double, 4> step_fractions{{0.50, 0.25, 0.125, 0.0625}};
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const std::vector<Point3> before_iteration = result.path;
    std::vector<Point3> proposal = before_iteration;
    std::size_t iteration_updates = 0;
    for (std::size_t index = 1; index + 1 < proposal.size(); ++index) {
      const Point3 previous = proposal[index - 1];
      const Point3 original = proposal[index];
      const Point3 next = proposal[index + 1];
      const Point3 midpoint{
        0.5 * (previous.x + next.x),
        0.5 * (previous.y + next.y),
        0.5 * (previous.z + next.z)};
      const std::vector<Point3> local_baseline{previous, original, next};
      const double local_baseline_clearance = minimumClearance(
        local_baseline, clearance, maximum_step);

      for (const double fraction : step_fractions) {
        const Point3 update = interpolate(original, midpoint, fraction);
        if (distance2d(update, original) <= 1.0e-10) {
          continue;
        }
        const std::vector<Point3> local_candidate{previous, update, next};
        const double local_candidate_clearance = minimumClearance(
          local_candidate, clearance, maximum_step);
        if (!segment_safe(previous, update) || !segment_safe(update, next) ||
          !segment_safe(original, update) ||
          !clearanceNotWorse(
            local_candidate_clearance, local_baseline_clearance, numeric_tolerance))
        {
          continue;
        }
        proposal[index] = update;
        ++iteration_updates;
        break;
      }
    }

    if (iteration_updates == 0) {
      break;
    }
    const ClearanceAssessment proposal_assessment = assess(
      raw_baseline, proposal, clearance, segment_safe,
      maximum_step, numeric_tolerance);
    if (!proposal_assessment.accepted) {
      break;
    }
    result.path = std::move(proposal);
    result.assessment = proposal_assessment;
    result.accepted_updates += iteration_updates;
    ++result.accepted_iterations;
  }
  result.changed = result.accepted_updates > 0;
  return result;
}

}  // namespace race_super_planner_ros2::smoothing

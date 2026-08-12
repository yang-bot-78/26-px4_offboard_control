#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Eigen>

namespace ego_planner
{

struct RecoveryPathPolicy
{
  struct PolylineProjection
  {
    double distance{std::numeric_limits<double>::infinity()};
    double arc_length{0.0};
    std::size_t segment{0};
    double ratio{0.0};
    Eigen::Vector3d point{Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN())};
  };

  struct RejoinPoint
  {
    bool found{false};
    Eigen::Vector3d point{Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN())};
    double forward_distance{0.0};
  };

  static double stableSampleSpacing(
    const double requested_spacing, const double map_resolution,
    const std::size_t required_consecutive_free)
  {
    if (!std::isfinite(requested_spacing) || requested_spacing <= 0.0 ||
      !std::isfinite(map_resolution) || map_resolution <= 0.0 ||
      required_consecutive_free == 0)
    {
      return std::numeric_limits<double>::quiet_NaN();
    }
    // Confirm free space over one full map voxel without limiting a short
    // recovery search to only a handful of coarse samples.
    return std::max(
      requested_spacing,
      map_resolution / static_cast<double>(required_consecutive_free));
  }

  static PolylineProjection projectToPolyline(
    const std::vector<Eigen::Vector3d> & polyline,
    const Eigen::Vector3d & query)
  {
    PolylineProjection best;
    if (!query.allFinite() || polyline.size() < 2) {
      return best;
    }

    double arc_before = 0.0;
    for (std::size_t index = 1; index < polyline.size(); ++index) {
      const Eigen::Vector2d a = polyline[index - 1].head<2>();
      const Eigen::Vector2d b = polyline[index].head<2>();
      if (!a.allFinite() || !b.allFinite()) {
        return PolylineProjection{};
      }
      const Eigen::Vector2d delta = b - a;
      const double length = delta.norm();
      if (length <= 1.0e-9) {
        continue;
      }
      const double ratio = std::clamp(
        (query.head<2>() - a).dot(delta) / delta.squaredNorm(), 0.0, 1.0);
      const Eigen::Vector3d projected =
        polyline[index - 1] + ratio * (polyline[index] - polyline[index - 1]);
      const double distance =
        (query.head<2>() - projected.head<2>()).norm();
      if (distance < best.distance) {
        best.distance = distance;
        best.arc_length = arc_before + ratio * length;
        best.segment = index - 1;
        best.ratio = ratio;
        best.point = projected;
      }
      arc_before += length;
    }
    return best;
  }

  static RejoinPoint findForwardRejoin(
    const Eigen::Vector3d & local_goal,
    const Eigen::Vector2d & forward,
    const double maximum_search_distance,
    const double sample_spacing,
    const std::size_t required_consecutive_free,
    const bool accept_free_local_goal,
    const std::function<bool(const Eigen::Vector3d &)> & is_free,
    const std::function<bool(const Eigen::Vector3d &)> & is_inside_bounds)
  {
    RejoinPoint result;
    if (!local_goal.allFinite() || !forward.allFinite() ||
      forward.norm() <= 1.0e-9 || !std::isfinite(maximum_search_distance) ||
      maximum_search_distance < 0.0 || !std::isfinite(sample_spacing) ||
      sample_spacing <= 0.0 || required_consecutive_free == 0 ||
      !is_free || !is_inside_bounds)
    {
      return result;
    }

    if (accept_free_local_goal && is_inside_bounds(local_goal) && is_free(local_goal)) {
      result.found = true;
      result.point = local_goal;
      return result;
    }

    const Eigen::Vector2d direction = forward.normalized();
    const std::size_t sample_count = static_cast<std::size_t>(
      std::ceil(maximum_search_distance / sample_spacing));
    std::size_t consecutive_free = 0;
    double previous_distance = -1.0;
    for (std::size_t index = 1; index <= sample_count; ++index) {
      const double distance = std::min(
        maximum_search_distance, static_cast<double>(index) * sample_spacing);
      if (distance <= previous_distance + 1.0e-9) {
        continue;
      }
      previous_distance = distance;
      Eigen::Vector3d candidate = local_goal;
      candidate.x() += direction.x() * distance;
      candidate.y() += direction.y() * distance;
      if (!is_inside_bounds(candidate)) {
        break;
      }
      if (!is_free(candidate)) {
        consecutive_free = 0;
        continue;
      }
      ++consecutive_free;
      if (consecutive_free >= required_consecutive_free) {
        result.found = true;
        result.point = candidate;
        result.forward_distance = distance;
        return result;
      }
    }
    return result;
  }

  static RejoinPoint findForwardRejoinOnPolyline(
    const std::vector<Eigen::Vector3d> & continuation,
    const double maximum_search_distance,
    const double sample_spacing,
    const std::size_t required_consecutive_free,
    const std::function<bool(const Eigen::Vector3d &)> & is_free,
    const std::function<bool(const Eigen::Vector3d &)> & is_inside_bounds)
  {
    RejoinPoint result;
    if (continuation.size() < 2 ||
      !std::isfinite(maximum_search_distance) || maximum_search_distance <= 0.0 ||
      !std::isfinite(sample_spacing) || sample_spacing <= 0.0 ||
      required_consecutive_free == 0 || !is_free || !is_inside_bounds)
    {
      return result;
    }

    std::vector<double> arc(continuation.size(), 0.0);
    for (std::size_t index = 1; index < continuation.size(); ++index) {
      if (!continuation[index - 1].allFinite() || !continuation[index].allFinite()) {
        return result;
      }
      arc[index] = arc[index - 1] +
        (continuation[index] - continuation[index - 1]).norm();
    }
    const double search_limit = std::min(maximum_search_distance, arc.back());
    if (search_limit <= 1.0e-9) {
      return result;
    }

    std::size_t segment = 1;
    std::size_t consecutive_free = 0;
    const std::size_t sample_count = static_cast<std::size_t>(
      std::ceil(search_limit / sample_spacing));
    double previous_distance = -1.0;
    for (std::size_t index = 1; index <= sample_count; ++index) {
      const double distance = std::min(
        search_limit, static_cast<double>(index) * sample_spacing);
      if (distance <= previous_distance + 1.0e-9) {
        continue;
      }
      previous_distance = distance;
      while (segment + 1 < arc.size() && arc[segment] < distance) {
        ++segment;
      }
      const double segment_length = arc[segment] - arc[segment - 1];
      if (segment_length <= 1.0e-9) {
        consecutive_free = 0;
        continue;
      }
      const double ratio = std::clamp(
        (distance - arc[segment - 1]) / segment_length, 0.0, 1.0);
      const Eigen::Vector3d candidate = continuation[segment - 1] +
        ratio * (continuation[segment] - continuation[segment - 1]);
      if (!is_inside_bounds(candidate)) {
        break;
      }
      if (!is_free(candidate)) {
        consecutive_free = 0;
        continue;
      }
      ++consecutive_free;
      if (consecutive_free >= required_consecutive_free) {
        result.found = true;
        result.point = candidate;
        result.forward_distance = distance;
        return result;
      }
    }
    return result;
  }

  struct LateralRejoinPoint
  {
    bool found{false};
    Eigen::Vector3d point{Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN())};
    double forward_distance{0.0};
    // Signed offset from the reference polyline, left of travel positive.  Only
    // reported for diagnostics; the planner never uses it to pick a side.
    double lateral_offset{0.0};
  };

  // Search for a rejoin point on lines parallel to the reference polyline.
  //
  // An obstacle sitting on the reference line occupies it for its own width
  // plus the clearance margin on both sides.  A 0.5m box at 0.40m clearance
  // blocks 1.3m of line, so a purely on-line forward search cannot find a free
  // point unless it is allowed to look further ahead than the blocked span.
  // Offsetting the search laterally reaches free space beside the obstacle at a
  // much shorter forward distance.
  //
  // This does NOT decide which way to go around.  Offsets are tried in
  // symmetric pairs at increasing magnitude, so the nearest reachable rejoin
  // point wins regardless of side, and the local A* still searches the actual
  // route to it.  It only supplies a target A* can reach.
  static LateralRejoinPoint findLateralRejoinOnPolyline(
    const std::vector<Eigen::Vector3d> & continuation,
    const double maximum_search_distance,
    const double sample_spacing,
    const std::size_t required_consecutive_free,
    const std::vector<double> & lateral_offsets,
    const std::function<bool(const Eigen::Vector3d &)> & is_free,
    const std::function<bool(const Eigen::Vector3d &)> & is_inside_bounds)
  {
    LateralRejoinPoint result;
    if (continuation.size() < 2 || lateral_offsets.empty() ||
      !std::isfinite(maximum_search_distance) || maximum_search_distance <= 0.0 ||
      !std::isfinite(sample_spacing) || sample_spacing <= 0.0 ||
      required_consecutive_free == 0 || !is_free || !is_inside_bounds)
    {
      return result;
    }

    // Prefer the smallest forward distance across all offsets rather than the
    // first offset that happens to work.  A rejoin point 0.3m ahead and 0.7m
    // aside is a tighter turn than one 1.2m ahead and 0.4m aside, and the
    // caller's forward-progress gate already rejects anything too close.
    double best_forward = std::numeric_limits<double>::infinity();
    for (const double offset : lateral_offsets) {
      if (!std::isfinite(offset)) {
        continue;
      }
      const auto shifted = offsetPolyline(continuation, offset);
      if (shifted.size() < 2) {
        continue;
      }
      const auto rejoin = findForwardRejoinOnPolyline(
        shifted, maximum_search_distance, sample_spacing,
        required_consecutive_free, is_free, is_inside_bounds);
      if (!rejoin.found || rejoin.forward_distance >= best_forward) {
        continue;
      }
      best_forward = rejoin.forward_distance;
      result.found = true;
      result.point = rejoin.point;
      result.forward_distance = rejoin.forward_distance;
      result.lateral_offset = offset;
    }
    return result;
  }

  // Shift a polyline sideways in the horizontal plane, keeping height.  Each
  // vertex moves along the left normal of its own local travel direction, so a
  // curved reference stays roughly parallel instead of collapsing at corners.
  static std::vector<Eigen::Vector3d> offsetPolyline(
    const std::vector<Eigen::Vector3d> & polyline, const double offset)
  {
    std::vector<Eigen::Vector3d> shifted;
    if (polyline.size() < 2 || !std::isfinite(offset)) {
      return shifted;
    }
    if (std::abs(offset) <= 1.0e-9) {
      return polyline;
    }
    shifted.reserve(polyline.size());
    for (std::size_t index = 0; index < polyline.size(); ++index) {
      if (!polyline[index].allFinite()) {
        return std::vector<Eigen::Vector3d>{};
      }
      // Average the incoming and outgoing directions at interior vertices so
      // the offset bisects the corner.
      Eigen::Vector2d direction = Eigen::Vector2d::Zero();
      if (index + 1 < polyline.size()) {
        direction += segmentDirection(polyline[index], polyline[index + 1]);
      }
      if (index > 0) {
        direction += segmentDirection(polyline[index - 1], polyline[index]);
      }
      if (direction.norm() <= 1.0e-9) {
        return std::vector<Eigen::Vector3d>{};
      }
      direction.normalize();
      const Eigen::Vector2d left_normal(-direction.y(), direction.x());
      Eigen::Vector3d point = polyline[index];
      point.x() += offset * left_normal.x();
      point.y() += offset * left_normal.y();
      shifted.push_back(point);
    }
    return shifted;
  }

  // Every lateral rejoin point that exists, ordered by ascending forward
  // distance.  findLateralRejoinOnPolyline returns only the best one, which is
  // not enough when that one turns out to be unroutable: the caller needs the
  // runners-up to fall through to.
  static std::vector<LateralRejoinPoint> findAllLateralRejoinsOnPolyline(
    const std::vector<Eigen::Vector3d> & continuation,
    const double maximum_search_distance,
    const double sample_spacing,
    const std::size_t required_consecutive_free,
    const std::vector<double> & lateral_offsets,
    const std::function<bool(const Eigen::Vector3d &)> & is_free,
    const std::function<bool(const Eigen::Vector3d &)> & is_inside_bounds)
  {
    std::vector<LateralRejoinPoint> found;
    if (continuation.size() < 2 || lateral_offsets.empty() ||
      !std::isfinite(maximum_search_distance) || maximum_search_distance <= 0.0 ||
      !std::isfinite(sample_spacing) || sample_spacing <= 0.0 ||
      required_consecutive_free == 0 || !is_free || !is_inside_bounds)
    {
      return found;
    }
    for (const double offset : lateral_offsets) {
      if (!std::isfinite(offset)) {
        continue;
      }
      const auto shifted = offsetPolyline(continuation, offset);
      if (shifted.size() < 2) {
        continue;
      }
      const auto rejoin = findForwardRejoinOnPolyline(
        shifted, maximum_search_distance, sample_spacing,
        required_consecutive_free, is_free, is_inside_bounds);
      if (!rejoin.found) {
        continue;
      }
      LateralRejoinPoint entry;
      entry.found = true;
      entry.point = rejoin.point;
      entry.forward_distance = rejoin.forward_distance;
      entry.lateral_offset = offset;
      found.push_back(entry);
    }
    std::stable_sort(
      found.begin(), found.end(),
      [](const LateralRejoinPoint & a, const LateralRejoinPoint & b) {
        return a.forward_distance < b.forward_distance;
      });
    return found;
  }

  // Symmetric offset ladder: nearest magnitude first, both sides at each step.
  // Ordering only affects tie-breaking; findLateralRejoinOnPolyline compares
  // forward distance across every entry.
  static std::vector<double> lateralOffsetLadder(
    const double maximum_offset, const double step)
  {
    std::vector<double> offsets;
    if (!std::isfinite(maximum_offset) || maximum_offset <= 0.0 ||
      !std::isfinite(step) || step <= 0.0)
    {
      return offsets;
    }
    for (double magnitude = step; magnitude <= maximum_offset + 1.0e-9;
      magnitude += step)
    {
      offsets.push_back(magnitude);
      offsets.push_back(-magnitude);
    }
    return offsets;
  }

private:
  static Eigen::Vector2d segmentDirection(
    const Eigen::Vector3d & from, const Eigen::Vector3d & to)
  {
    const Eigen::Vector2d delta = (to - from).head<2>();
    const double length = delta.norm();
    return length <= 1.0e-9 ? Eigen::Vector2d::Zero() : Eigen::Vector2d(delta / length);
  }
};

}  // namespace ego_planner

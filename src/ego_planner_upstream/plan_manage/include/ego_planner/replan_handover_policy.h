#ifndef EGO_PLANNER_REPLAN_HANDOVER_POLICY_H_
#define EGO_PLANNER_REPLAN_HANDOVER_POLICY_H_

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ego_planner
{

struct ReferencePrefixTrimResult
{
  std::vector<Eigen::Vector3d> points;
  std::size_t original_points{0};
  std::size_t projection_segment{0};
  double projection_ratio{0.0};
  double projection_distance{std::numeric_limits<double>::infinity()};
  double discarded_length{0.0};
  double remaining_length{0.0};
  bool valid{false};
};

struct HandoverContinuityResult
{
  double position_error{std::numeric_limits<double>::infinity()};
  double velocity_error{std::numeric_limits<double>::infinity()};
  double acceleration_error{std::numeric_limits<double>::infinity()};
  bool accepted{false};
};

struct HandoverRejectAction
{
  bool keep_active{false};
  bool publish_fatal_collision{true};
  double retry_delay_sec{0.0};
};

class ReplanHandoverPolicy
{
public:
  static double effectiveOdomReanchorThreshold(
    const double configured_threshold, const double horizontal_speed,
    const double maximum_threshold = 0.30)
  {
    if (!std::isfinite(configured_threshold) || configured_threshold < 0.0 ||
      !std::isfinite(maximum_threshold) || maximum_threshold < configured_threshold)
    {
      return configured_threshold;
    }
    // Allow one short control/planning reaction window of normal tracking lag.
    // The hard cap still guarantees that a real divergence is re-anchored.
    constexpr double reaction_window_sec = 0.05;
    const double speed = std::max(0.0, std::isfinite(horizontal_speed) ? horizontal_speed : 0.0);
    return std::min(maximum_threshold, configured_threshold + reaction_window_sec * speed);
  }

  static bool requiresOdomReanchor(
    const double horizontal_position_error, const double threshold)
  {
    return std::isfinite(horizontal_position_error) && std::isfinite(threshold) &&
           threshold >= 0.0 && horizontal_position_error > threshold;
  }

  // An odometry re-anchor means the active trajectory's state is no longer
  // the vehicle state. Its remaining map clearance cannot justify continuing
  // to command it after the replacement plan fails.
  static bool mayPreserveActiveTrajectoryAfterFailure(
    const bool reanchoring_from_odom, const bool active_remaining_safe)
  {
    return !reanchoring_from_odom && active_remaining_safe;
  }

  static bool isTrajectoryActiveAt(
    const double trajectory_start_sec, const double trajectory_duration_sec,
    const double handover_time_sec)
  {
    return std::isfinite(trajectory_start_sec) &&
           std::isfinite(trajectory_duration_sec) &&
           std::isfinite(handover_time_sec) &&
           trajectory_start_sec > 1.0e-5 && trajectory_duration_sec > 0.0 &&
           handover_time_sec < trajectory_start_sec + trajectory_duration_sec;
  }

  static bool requiresMeasuredOdomStart(
    const double trajectory_start_sec, const double trajectory_duration_sec,
    const double trajectory_time_sec, const double horizontal_position_error,
    const double reanchor_threshold)
  {
    return !isTrajectoryActiveAt(
      trajectory_start_sec, trajectory_duration_sec, trajectory_time_sec) ||
           requiresOdomReanchor(horizontal_position_error, reanchor_threshold);
  }

  static ReferencePrefixTrimResult trimReferencePrefix(
    const std::vector<Eigen::Vector3d> & reference,
    const Eigen::Vector3d & start)
  {
    ReferencePrefixTrimResult result;
    result.original_points = reference.size();
    if (!start.allFinite()) {
      return result;
    }

    std::vector<Eigen::Vector3d> clean;
    clean.reserve(reference.size());
    for (const auto & point : reference) {
      if (!point.allFinite()) {
        continue;
      }
      if (clean.empty() || (point - clean.back()).norm() > 1.0e-6) {
        clean.push_back(point);
      }
    }
    if (clean.size() < 2) {
      return result;
    }

    std::vector<double> cumulative(clean.size(), 0.0);
    for (std::size_t index = 1; index < clean.size(); ++index) {
      cumulative[index] =
        cumulative[index - 1] + (clean[index] - clean[index - 1]).norm();
    }

    Eigen::Vector3d best_projection = clean.front();
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_segment = 0;
    double best_ratio = 0.0;
    for (std::size_t segment = 0; segment + 1 < clean.size(); ++segment) {
      const Eigen::Vector3d delta = clean[segment + 1] - clean[segment];
      const double length_squared = delta.squaredNorm();
      if (length_squared <= 1.0e-12) {
        continue;
      }
      const double ratio = std::clamp(
        (start - clean[segment]).dot(delta) / length_squared, 0.0, 1.0);
      const Eigen::Vector3d projection = clean[segment] + ratio * delta;
      const double distance = (start - projection).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_projection = projection;
        best_segment = segment;
        best_ratio = ratio;
      }
    }
    if (!std::isfinite(best_distance)) {
      return result;
    }

    result.points.reserve(clean.size() - best_segment);
    result.points.push_back(best_projection);
    for (std::size_t index = best_segment + 1; index < clean.size(); ++index) {
      if ((clean[index] - result.points.back()).norm() > 1.0e-6) {
        result.points.push_back(clean[index]);
      }
    }
    if (result.points.size() < 2) {
      return result;
    }

    const double segment_length = (clean[best_segment + 1] - clean[best_segment]).norm();
    result.projection_segment = best_segment;
    result.projection_ratio = best_ratio;
    result.projection_distance = best_distance;
    result.discarded_length = cumulative[best_segment] + best_ratio * segment_length;
    result.remaining_length = cumulative.back() - result.discarded_length;
    result.valid = std::isfinite(result.remaining_length) && result.remaining_length > 1.0e-6;
    return result;
  }

  static HandoverContinuityResult validate(
    const Eigen::Vector3d & expected_position,
    const Eigen::Vector3d & expected_velocity,
    const Eigen::Vector3d & expected_acceleration,
    const Eigen::Vector3d & candidate_position,
    const Eigen::Vector3d & candidate_velocity,
    const Eigen::Vector3d & candidate_acceleration,
    const double position_threshold = 0.001,
    const double velocity_threshold = 0.001,
    const double acceleration_threshold = 0.005)
  {
    HandoverContinuityResult result;
    if (!expected_position.allFinite() || !expected_velocity.allFinite() ||
      !expected_acceleration.allFinite() || !candidate_position.allFinite() ||
      !candidate_velocity.allFinite() || !candidate_acceleration.allFinite())
    {
      return result;
    }
    result.position_error = (candidate_position - expected_position).norm();
    result.velocity_error = (candidate_velocity - expected_velocity).norm();
    result.acceleration_error = (candidate_acceleration - expected_acceleration).norm();
    result.accepted =
      result.position_error <= position_threshold &&
      result.velocity_error <= velocity_threshold &&
      result.acceleration_error <= acceleration_threshold;
    return result;
  }

  static HandoverRejectAction rejectedCandidateAction(const bool active_remaining_safe)
  {
    if (active_remaining_safe) {
      return HandoverRejectAction{true, false, 0.25};
    }
    return HandoverRejectAction{false, true, 0.0};
  }
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_REPLAN_HANDOVER_POLICY_H_

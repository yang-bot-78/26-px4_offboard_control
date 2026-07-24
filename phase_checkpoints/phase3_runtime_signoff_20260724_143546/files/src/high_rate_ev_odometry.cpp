#include "high_rate_ev_odometry.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>

namespace fast_lio
{
namespace
{

constexpr double kMinimumPublishInterval = 0.020;
constexpr double kMaximumPublishInterval = 1.0 / 30.0;

Eigen::Matrix3d skew(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d result;
  result <<
    0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return result;
}

template<typename Derived>
bool finite_matrix(const Eigen::MatrixBase<Derived> & matrix)
{
  return matrix.array().isFinite().all();
}

template<typename MatrixT>
bool symmetric_psd(
  const MatrixT & matrix, double symmetry_tolerance, double psd_tolerance)
{
  if (!finite_matrix(matrix)) {return false;}
  if ((matrix - matrix.transpose()).cwiseAbs().maxCoeff() > symmetry_tolerance) {
    return false;
  }
  const MatrixT symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<MatrixT> solver(symmetric, Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success &&
    solver.eigenvalues().minCoeff() >= -psd_tolerance;
}

}  // namespace

ExactImuStampIndex::ExactImuStampIndex(std::size_t capacity)
: capacity_(capacity)
{
}

bool ExactImuStampIndex::valid() const
{
  return capacity_ > 0;
}

bool ExactImuStampIndex::record(double timestamp, std::int64_t nanoseconds)
{
  if (!valid() || !std::isfinite(timestamp) || timestamp < 0.0 ||
    nanoseconds < 0)
  {
    return false;
  }
  if (!entries_.empty() && timestamp <= entries_.back().timestamp) {
    return false;
  }
  entries_.push_back(Entry{timestamp, nanoseconds});
  while (entries_.size() > capacity_) {entries_.pop_front();}
  return true;
}

bool ExactImuStampIndex::lookup(
  double timestamp, std::int64_t * nanoseconds) const
{
  if (nanoseconds == nullptr || !std::isfinite(timestamp) || timestamp < 0.0) {
    return false;
  }
  for (auto entry = entries_.rbegin(); entry != entries_.rend(); ++entry) {
    if (entry->timestamp == timestamp) {
      *nanoseconds = entry->nanoseconds;
      return true;
    }
    if (entry->timestamp < timestamp) {break;}
  }
  return false;
}

HighRateEvOdometryGate::HighRateEvOdometryGate()
: HighRateEvOdometryGate(Config())
{
}

HighRateEvOdometryGate::HighRateEvOdometryGate(const Config & config)
: config_(config), valid_(validate_config())
{
}

bool HighRateEvOdometryGate::enabled() const
{
  return config_.enabled;
}

bool HighRateEvOdometryGate::valid() const
{
  return valid_;
}

bool HighRateEvOdometryGate::validate_config() const
{
  return std::isfinite(config_.publish_interval_s) &&
    config_.publish_interval_s >= kMinimumPublishInterval &&
    config_.publish_interval_s <= kMaximumPublishInterval &&
    std::isfinite(config_.unknown_angular_velocity_variance) &&
    config_.unknown_angular_velocity_variance > 0.0 &&
    std::isfinite(config_.covariance_symmetry_tolerance) &&
    config_.covariance_symmetry_tolerance > 0.0 &&
    std::isfinite(config_.covariance_psd_tolerance) &&
    config_.covariance_psd_tolerance > 0.0;
}

bool HighRateEvOdometryGate::pair_is_publishable(
  const HighRateEvPropagator::Candidate & candidate) const
{
  if (!config_.enabled || !valid_ || candidate.generation == 0 ||
    candidate.imu_sequence == 0 || !std::isfinite(candidate.timestamp) ||
    candidate.timestamp < 0.0)
  {
    return false;
  }
  if (!have_published_pair_) {return true;}
  return candidate.imu_sequence > last_published_sequence_ &&
    candidate.timestamp > last_published_timestamp_ &&
    candidate.timestamp - last_published_timestamp_ + 1.0e-12 >=
    config_.publish_interval_s;
}

bool HighRateEvOdometryGate::map_candidate(
  const HighRateEvPropagator::Candidate & candidate,
  PropagatedOdometry * output) const
{
  if (output == nullptr || !finite_matrix(candidate.covariance) ||
    !symmetric_psd(
      candidate.covariance, config_.covariance_symmetry_tolerance,
      config_.covariance_psd_tolerance))
  {
    return false;
  }

  const Eigen::Matrix<double, 23, 23> covariance =
    0.5 * (candidate.covariance + candidate.covariance.transpose());
  const Eigen::Matrix3d rotation_world_body = candidate.state.rot.toRotationMatrix();
  if (!finite_matrix(rotation_world_body) || !finite_matrix(candidate.state.pos) ||
    !finite_matrix(candidate.state.vel))
  {
    return false;
  }

  Eigen::Matrix<double, 6, 23> pose_jacobian =
    Eigen::Matrix<double, 6, 23>::Zero();
  pose_jacobian.block<3, 3>(0, 0).setIdentity();
  pose_jacobian.block<3, 3>(3, 3) = rotation_world_body;
  Eigen::Matrix<double, 6, 6> pose_covariance =
    pose_jacobian * covariance * pose_jacobian.transpose();
  pose_covariance = 0.5 * (pose_covariance + pose_covariance.transpose());

  const Eigen::Vector3d velocity_body =
    rotation_world_body.transpose() * candidate.state.vel;
  Eigen::Matrix<double, 3, 23> velocity_jacobian =
    Eigen::Matrix<double, 3, 23>::Zero();
  velocity_jacobian.block<3, 3>(0, 3) = skew(velocity_body);
  velocity_jacobian.block<3, 3>(0, 12) = rotation_world_body.transpose();
  Eigen::Matrix3d velocity_covariance =
    velocity_jacobian * covariance * velocity_jacobian.transpose();
  velocity_covariance =
    0.5 * (velocity_covariance + velocity_covariance.transpose());

  if (!finite_matrix(velocity_body) ||
    !symmetric_psd(
      pose_covariance, config_.covariance_symmetry_tolerance,
      config_.covariance_psd_tolerance) ||
    !symmetric_psd(
      velocity_covariance, config_.covariance_symmetry_tolerance,
      config_.covariance_psd_tolerance))
  {
    return false;
  }
  for (int axis = 0; axis < 6; ++axis) {
    if (!(pose_covariance(axis, axis) > 0.0)) {return false;}
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!(velocity_covariance(axis, axis) > 0.0)) {return false;}
  }

  PropagatedOdometry prepared;
  prepared.generation = candidate.generation;
  prepared.parent_generation = candidate.parent_generation;
  prepared.validation_epoch = candidate.validation_epoch;
  prepared.imu_sequence = candidate.imu_sequence;
  prepared.timestamp = candidate.timestamp;
  prepared.position_world = candidate.state.pos;
  prepared.orientation_world_body = Eigen::Quaterniond(rotation_world_body).normalized();
  prepared.linear_velocity_body = velocity_body;
  prepared.angular_velocity_body =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  prepared.pose_covariance.fill(0.0);
  prepared.twist_covariance.fill(0.0);
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      prepared.pose_covariance[static_cast<std::size_t>(row * 6 + column)] =
        pose_covariance(row, column);
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      prepared.twist_covariance[static_cast<std::size_t>(row * 6 + column)] =
        velocity_covariance(row, column);
    }
  }
  prepared.twist_covariance[21] = config_.unknown_angular_velocity_variance;
  prepared.twist_covariance[28] = config_.unknown_angular_velocity_variance;
  prepared.twist_covariance[35] = config_.unknown_angular_velocity_variance;
  if (!finite_matrix(prepared.orientation_world_body.coeffs())) {return false;}
  *output = prepared;
  return true;
}

bool HighRateEvOdometryGate::prepare(
  const std::shared_ptr<const HighRateEvPropagator::Candidate> & candidate,
  PropagatedOdometry * output) const
{
  return candidate && pair_is_publishable(*candidate) &&
    map_candidate(*candidate, output);
}

bool HighRateEvOdometryGate::commit(
  const std::shared_ptr<const HighRateEvPropagator::Candidate> & candidate)
{
  if (!candidate || !pair_is_publishable(*candidate)) {return false;}
  have_published_pair_ = true;
  last_published_sequence_ = candidate->imu_sequence;
  last_published_timestamp_ = candidate->timestamp;
  return true;
}

bool HighRateEvOdometryGate::try_build(
  const std::shared_ptr<const HighRateEvPropagator::Candidate> & candidate,
  PropagatedOdometry * output)
{
  return prepare(candidate, output) && commit(candidate);
}

std::unique_ptr<HighRateEvOdometryGate> make_high_rate_ev_odometry_gate_if_enabled(
  const HighRateEvOdometryGate::Config & config)
{
  if (!config.enabled) {return std::unique_ptr<HighRateEvOdometryGate>();}
  std::unique_ptr<HighRateEvOdometryGate> gate(new HighRateEvOdometryGate(config));
  if (!gate->valid()) {return std::unique_ptr<HighRateEvOdometryGate>();}
  return gate;
}

}  // namespace fast_lio

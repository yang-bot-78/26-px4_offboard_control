// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <fr_lio/so3_math.hpp>

namespace fr_lio
{

enum class AccelerationUnit
{
  Unconfirmed,
  MetersPerSecondSquared,
  Gravity,
};

inline std::optional<AccelerationUnit> parse_acceleration_unit(const std::string & value)
{
  if (value == "unconfirmed") {
    return AccelerationUnit::Unconfirmed;
  }
  if (value == "mps2") {
    return AccelerationUnit::MetersPerSecondSquared;
  }
  if (value == "g") {
    return AccelerationUnit::Gravity;
  }
  return std::nullopt;
}

inline double acceleration_scale(AccelerationUnit unit, double standard_gravity = 9.81)
{
  return unit == AccelerationUnit::Gravity ? standard_gravity : 1.0;
}

class AccelerationUnitReport
{
public:
  explicit AccelerationUnitReport(double duration_s = 10.0)
  : duration_s_(duration_s)
  {
  }

  void add(double timestamp, const Eigen::Vector3d & acceleration)
  {
    if (!std::isfinite(timestamp) || !acceleration.allFinite()) {
      return;
    }
    if (!started_) {
      start_timestamp_ = timestamp;
      started_ = true;
    }
    last_timestamp_ = timestamp;
    ++count_;
    const double norm = acceleration.norm();
    const double delta = norm - mean_norm_;
    mean_norm_ += delta / static_cast<double>(count_);
    m2_norm_ += delta * (norm - mean_norm_);
  }

  bool ready() const
  {
    return started_ && last_timestamp_ - start_timestamp_ >= duration_s_;
  }

  std::size_t count() const {return count_;}
  double mean_norm() const {return mean_norm_;}
  double stddev_norm() const
  {
    return count_ > 1 ? std::sqrt(m2_norm_ / static_cast<double>(count_ - 1)) : 0.0;
  }

  std::string suggested_unit() const
  {
    if (!ready()) {
      return "insufficient_data";
    }
    if (std::abs(mean_norm_ - 1.0) <= 0.25) {
      return "g";
    }
    if (std::abs(mean_norm_ - 9.81) <= 2.0) {
      return "mps2";
    }
    return "unknown";
  }

private:
  double duration_s_{10.0};
  double start_timestamp_{0.0};
  double last_timestamp_{0.0};
  double mean_norm_{0.0};
  double m2_norm_{0.0};
  std::size_t count_{0};
  bool started_{false};
};

struct HighRateImuSample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp{0.0};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct HighRateOdomState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int kErrorSize = 18;

  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, 0.0, -9.81)};
  Eigen::Matrix<double, kErrorSize, kErrorSize> covariance{
    Eigen::Matrix<double, kErrorSize, kErrorSize>::Zero()};
  double timestamp{0.0};
};

struct HighRateOdomNoise
{
  double gyro_variance{0.1};
  double accel_variance{0.1};
  double gyro_bias_variance{0.0001};
  double accel_bias_variance{0.0001};
};

enum class HighRateOdomHealth
{
  Healthy,
  Suspect,
  StaleLidar,
};

struct HighRateOdomResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HighRateOdomState state;
  Eigen::Vector3d body_angular_velocity{Eigen::Vector3d::Zero()};
  double lidar_anchor_age_s{0.0};
  HighRateOdomHealth health{HighRateOdomHealth::Healthy};
  bool publish{false};
};

class HighRateOdomPropagator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HighRateOdomPropagator(
    HighRateOdomNoise noise = {}, double warn_anchor_age_s = 0.15,
    double max_anchor_age_s = 0.40, double history_duration_s = 2.0)
  : noise_(noise),
    warn_anchor_age_s_(warn_anchor_age_s),
    max_anchor_age_s_(max_anchor_age_s),
    history_duration_s_(history_duration_s)
  {
  }

  bool reset_from_lidar(const HighRateOdomState & corrected_state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_is_valid(corrected_state) ||
      (initialized_ && corrected_state.timestamp <= lidar_anchor_timestamp_) ||
      (!history_.empty() && corrected_state.timestamp > history_.back().timestamp))
    {
      return false;
    }

    state_ = corrected_state;
    state_.rotation = orthonormalized(state_.rotation);
    state_.covariance = sanitized_covariance(state_.covariance);
    lidar_anchor_timestamp_ = corrected_state.timestamp;
    initialized_ = true;

    previous_input_ = input_at_locked(corrected_state.timestamp);
    if (!previous_input_) {
      return true;
    }

    for (const auto & sample : history_) {
      if (sample.timestamp <= corrected_state.timestamp) {
        continue;
      }
      propagate_locked(sample);
    }
    return true;
  }

  std::optional<HighRateOdomResult> add_imu(const HighRateImuSample & sample)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sample_is_valid(sample)) {
      return std::nullopt;
    }

    if (!history_.empty() && sample.timestamp <= history_.back().timestamp) {
      if (sample.timestamp < history_.back().timestamp) {
        history_.clear();
        previous_input_.reset();
        initialized_ = false;
      }
      return std::nullopt;
    }

    history_.push_back(sample);
    prune_history_locked(sample.timestamp);
    if (!initialized_) {
      return std::nullopt;
    }

    if (!previous_input_) {
      previous_input_ = sample;
      previous_input_->timestamp = state_.timestamp;
      return std::nullopt;
    }
    if (sample.timestamp <= state_.timestamp) {
      return std::nullopt;
    }

    propagate_locked(sample);
    return make_result_locked();
  }

  std::optional<HighRateOdomResult> current() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || !previous_input_) {
      return std::nullopt;
    }
    return make_result_locked();
  }

  void invalidate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = false;
    previous_input_.reset();
  }

  static Eigen::Matrix<double, 6, 6> pose_covariance(const HighRateOdomState & state)
  {
    Eigen::Matrix<double, 6, HighRateOdomState::kErrorSize> jacobian =
      Eigen::Matrix<double, 6, HighRateOdomState::kErrorSize>::Zero();
    jacobian.block<3, 3>(0, 0).setIdentity();
    // The propagated attitude error is a right/body tangent perturbation.
    // nav_msgs/Odometry pose covariance is expressed in header.frame_id.
    jacobian.block<3, 3>(3, 3) = state.rotation;
    return sanitize_output_covariance(jacobian * state.covariance * jacobian.transpose());
  }

  static Eigen::Matrix<double, 6, 6> twist_covariance(
    const HighRateOdomState & state, const Eigen::Vector3d & body_angular_velocity,
    double gyro_measurement_variance)
  {
    const Eigen::Vector3d velocity_body = state.rotation.transpose() * state.velocity;
    Eigen::Matrix<double, 6, HighRateOdomState::kErrorSize> jacobian =
      Eigen::Matrix<double, 6, HighRateOdomState::kErrorSize>::Zero();
    jacobian.block<3, 3>(0, 3) = skew_sym_mat(velocity_body);
    jacobian.block<3, 3>(0, 6) = state.rotation.transpose();
    jacobian.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> covariance =
      jacobian * state.covariance * jacobian.transpose();
    covariance.block<3, 3>(3, 3).diagonal().array() +=
      std::max(0.0, gyro_measurement_variance);
    (void)body_angular_velocity;
    return sanitize_output_covariance(covariance);
  }

private:
  using Covariance = Eigen::Matrix<double, HighRateOdomState::kErrorSize,
    HighRateOdomState::kErrorSize>;

  static bool sample_is_valid(const HighRateImuSample & sample)
  {
    return std::isfinite(sample.timestamp) && sample.acceleration.allFinite() &&
           sample.angular_velocity.allFinite();
  }

  static bool state_is_valid(const HighRateOdomState & state)
  {
    return std::isfinite(state.timestamp) && state.position.allFinite() &&
           state.rotation.allFinite() && state.velocity.allFinite() &&
           state.gyro_bias.allFinite() && state.accel_bias.allFinite() &&
           state.gravity.allFinite() && state.covariance.allFinite();
  }

  static Eigen::Matrix3d orthonormalized(const Eigen::Matrix3d & rotation)
  {
    Eigen::Quaterniond quaternion(rotation);
    quaternion.normalize();
    return quaternion.toRotationMatrix();
  }

  static Covariance sanitized_covariance(const Covariance & covariance)
  {
    Covariance output = 0.5 * (covariance + covariance.transpose());
    for (int index = 0; index < output.rows(); ++index) {
      output(index, index) = std::max(output(index, index), 1e-12);
    }
    return output;
  }

  static Eigen::Matrix<double, 6, 6> sanitize_output_covariance(
    const Eigen::Matrix<double, 6, 6> & covariance)
  {
    Eigen::Matrix<double, 6, 6> output = 0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(output);
    if (solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() < 0.0) {
      auto eigenvalues = solver.eigenvalues().cwiseMax(1e-12);
      output = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
    }
    for (int index = 0; index < output.rows(); ++index) {
      output(index, index) = std::max(output(index, index), 1e-12);
    }
    return output;
  }

  std::optional<HighRateImuSample> input_at_locked(double timestamp) const
  {
    if (history_.empty()) {
      return std::nullopt;
    }

    const HighRateImuSample * before = nullptr;
    const HighRateImuSample * after = nullptr;
    for (const auto & sample : history_) {
      if (sample.timestamp <= timestamp) {
        before = &sample;
      }
      if (sample.timestamp >= timestamp) {
        after = &sample;
        break;
      }
    }

    if (before && after && after->timestamp > before->timestamp) {
      const double alpha = (timestamp - before->timestamp) /
        (after->timestamp - before->timestamp);
      HighRateImuSample interpolated;
      interpolated.timestamp = timestamp;
      interpolated.acceleration =
        (1.0 - alpha) * before->acceleration + alpha * after->acceleration;
      interpolated.angular_velocity =
        (1.0 - alpha) * before->angular_velocity + alpha * after->angular_velocity;
      return interpolated;
    }

    HighRateImuSample nearest = before ? *before : *after;
    nearest.timestamp = timestamp;
    return nearest;
  }

  void propagate_locked(const HighRateImuSample & sample)
  {
    const double dt = sample.timestamp - state_.timestamp;
    if (dt <= 0.0 || !previous_input_) {
      return;
    }

    const Eigen::Vector3d omega_previous =
      previous_input_->angular_velocity - state_.gyro_bias;
    const Eigen::Vector3d omega_current = sample.angular_velocity - state_.gyro_bias;
    const Eigen::Vector3d omega = 0.5 * (omega_previous + omega_current);
    const Eigen::Vector3d accel_previous =
      previous_input_->acceleration - state_.accel_bias;
    const Eigen::Vector3d accel_current = sample.acceleration - state_.accel_bias;
    const Eigen::Vector3d accel_body = 0.5 * (accel_previous + accel_current);

    const Eigen::Matrix3d rotation_previous = state_.rotation;
    const Eigen::Matrix3d rotation_current =
      rotation_previous * Exp(omega, dt);
    const Eigen::Vector3d acceleration_world_previous =
      rotation_previous * accel_previous + state_.gravity;
    const Eigen::Vector3d acceleration_world_current =
      rotation_current * accel_current + state_.gravity;
    const Eigen::Vector3d acceleration_world =
      0.5 * (acceleration_world_previous + acceleration_world_current);

    const Eigen::Vector3d velocity_previous = state_.velocity;
    state_.position += velocity_previous * dt + 0.5 * acceleration_world * dt * dt;
    state_.velocity += acceleration_world * dt;
    state_.rotation = orthonormalized(rotation_current);

    Covariance transition = Covariance::Identity();
    transition.block<3, 3>(0, 3) =
      -0.5 * rotation_previous * skew_sym_mat(accel_body) * dt * dt;
    transition.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    transition.block<3, 3>(0, 12) = -0.5 * rotation_previous * dt * dt;
    transition.block<3, 3>(0, 15) = 0.5 * Eigen::Matrix3d::Identity() * dt * dt;
    transition.block<3, 3>(3, 3) -= skew_sym_mat(omega) * dt;
    transition.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity() * dt;
    transition.block<3, 3>(6, 3) =
      -rotation_previous * skew_sym_mat(accel_body) * dt;
    transition.block<3, 3>(6, 12) = -rotation_previous * dt;
    transition.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;

    Covariance process_noise = Covariance::Zero();
    const double gyro_variance = std::max(0.0, noise_.gyro_variance);
    const double accel_variance = std::max(0.0, noise_.accel_variance);
    process_noise.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * accel_variance * dt * dt * dt / 3.0;
    process_noise.block<3, 3>(0, 6) =
      Eigen::Matrix3d::Identity() * accel_variance * dt * dt / 2.0;
    process_noise.block<3, 3>(6, 0) = process_noise.block<3, 3>(0, 6);
    process_noise.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * gyro_variance * dt;
    process_noise.block<3, 3>(6, 6) =
      Eigen::Matrix3d::Identity() * accel_variance * dt;
    process_noise.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() *
      std::max(0.0, noise_.gyro_bias_variance) * dt;
    process_noise.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() *
      std::max(0.0, noise_.accel_bias_variance) * dt;

    state_.covariance = sanitized_covariance(
      transition * state_.covariance * transition.transpose() + process_noise);
    state_.timestamp = sample.timestamp;
    previous_input_ = sample;
    last_body_angular_velocity_ = omega_current;
  }

  HighRateOdomResult make_result_locked() const
  {
    HighRateOdomResult result;
    result.state = state_;
    result.body_angular_velocity = last_body_angular_velocity_;
    result.lidar_anchor_age_s = state_.timestamp - lidar_anchor_timestamp_;
    if (result.lidar_anchor_age_s > max_anchor_age_s_) {
      result.health = HighRateOdomHealth::StaleLidar;
      result.publish = false;
    } else if (result.lidar_anchor_age_s > warn_anchor_age_s_) {
      result.health = HighRateOdomHealth::Suspect;
      result.publish = true;
    } else {
      result.health = HighRateOdomHealth::Healthy;
      result.publish = true;
    }
    return result;
  }

  void prune_history_locked(double newest_timestamp)
  {
    const double oldest_allowed = newest_timestamp - history_duration_s_;
    while (history_.size() > 2 && history_[1].timestamp < oldest_allowed) {
      history_.pop_front();
    }
  }

  HighRateOdomNoise noise_;
  double warn_anchor_age_s_{0.15};
  double max_anchor_age_s_{0.40};
  double history_duration_s_{2.0};
  mutable std::mutex mutex_;
  std::deque<HighRateImuSample, Eigen::aligned_allocator<HighRateImuSample>> history_;
  HighRateOdomState state_;
  std::optional<HighRateImuSample> previous_input_;
  Eigen::Vector3d last_body_angular_velocity_{Eigen::Vector3d::Zero()};
  double lidar_anchor_timestamp_{0.0};
  bool initialized_{false};
};

}  // namespace fr_lio

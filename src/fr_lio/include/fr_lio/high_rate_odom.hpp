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
#include <vector>

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
  Eigen::Vector3d raw_acceleration{Eigen::Vector3d::Zero()};
  bool accel_spike_rejected{false};
  bool accel_norm_limit_exceeded{false};
  bool accel_deviation_limit_exceeded{false};
  double raw_acceleration_norm_mps2{0.0};
  double acceleration_deviation_mps2{0.0};
  std::size_t accel_spike_rejection_count{0};
  std::size_t consecutive_accel_spike_rejections{0};
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

struct HighRateAccelFilterConfig
{
  bool enabled{true};
  std::size_t window_size{21};
  std::size_t min_samples{7};
  double max_deviation_mps2{5.0};
  double max_norm_mps2{19.62};
};

// Produces the single IMU sample that every estimator path must consume.
// The raw value and rejection decision are retained as metadata for logging
// and for scaling the ESKF process noise without dropping the sample time.
class HighRateImuPreprocessor
{
public:
  explicit HighRateImuPreprocessor(HighRateAccelFilterConfig config = {})
  : config_(config)
  {
    config_.window_size = std::max<std::size_t>(3, config_.window_size);
    if (config_.window_size % 2 == 0) ++config_.window_size;
    config_.min_samples = std::clamp<std::size_t>(
      config_.min_samples, 1, config_.window_size);
  }

  HighRateImuSample process(const HighRateImuSample & input)
  {
    HighRateImuSample used = input;
    used.raw_acceleration = input.acceleration;
    used.raw_acceleration_norm_mps2 = input.acceleration.norm();
    used.acceleration_deviation_mps2 = 0.0;
    used.accel_spike_rejected = false;
    used.accel_norm_limit_exceeded = false;
    used.accel_deviation_limit_exceeded = false;

    if (config_.enabled) {
      const bool has_reference = !used_acceleration_history_.empty();
      const Eigen::Vector3d reference = has_reference ? median() : input.acceleration;
      used.acceleration_deviation_mps2 = (input.acceleration - reference).norm();
      used.accel_norm_limit_exceeded = config_.max_norm_mps2 > 0.0 &&
        used.raw_acceleration_norm_mps2 > config_.max_norm_mps2;
      used.accel_deviation_limit_exceeded =
        used_acceleration_history_.size() >= config_.min_samples &&
        config_.max_deviation_mps2 > 0.0 &&
        used.acceleration_deviation_mps2 > config_.max_deviation_mps2;

      bool sustained_candidate = false;
      if (used.accel_norm_limit_exceeded) {
        candidate_.reset();
        candidate_count_ = 0;
        sustained_confirmed_ = false;
      } else if (used.accel_deviation_limit_exceeded) {
        const bool matches = candidate_.has_value() &&
          (input.acceleration - *candidate_).norm() <= config_.max_deviation_mps2;
        if (!matches) {
          candidate_ = input.acceleration;
          candidate_count_ = 1;
          sustained_confirmed_ = false;
        } else {
          ++candidate_count_;
        }
        if (sustained_confirmed_ || candidate_count_ > config_.min_samples) {
          sustained_confirmed_ = true;
          sustained_candidate = true;
        }
      } else {
        candidate_.reset();
        candidate_count_ = 0;
        sustained_confirmed_ = false;
      }

      used.accel_spike_rejected = has_reference &&
        (used.accel_norm_limit_exceeded ||
        (used.accel_deviation_limit_exceeded && !sustained_candidate));
      if (used.accel_spike_rejected) {
        used.acceleration = reference;
        ++rejection_count_;
        ++consecutive_rejections_;
      } else {
        consecutive_rejections_ = 0;
      }
    } else {
      candidate_.reset();
      candidate_count_ = 0;
      sustained_confirmed_ = false;
      consecutive_rejections_ = 0;
    }

    used.accel_spike_rejection_count = rejection_count_;
    used.consecutive_accel_spike_rejections = consecutive_rejections_;
    used_acceleration_history_.push_back(used.acceleration);
    while (used_acceleration_history_.size() > config_.window_size) {
      used_acceleration_history_.pop_front();
    }
    return used;
  }

  void reset()
  {
    used_acceleration_history_.clear();
    candidate_.reset();
    candidate_count_ = 0;
    sustained_confirmed_ = false;
    rejection_count_ = 0;
    consecutive_rejections_ = 0;
  }

private:
  Eigen::Vector3d median() const
  {
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    std::vector<double> values;
    values.reserve(used_acceleration_history_.size());
    for (int axis = 0; axis < 3; ++axis) {
      values.clear();
      for (const auto & value : used_acceleration_history_) values.push_back(value[axis]);
      std::sort(values.begin(), values.end());
      const std::size_t middle = values.size() / 2;
      result[axis] = values[middle];
      if (values.size() % 2 == 0) result[axis] = 0.5 * (result[axis] + values[middle - 1]);
    }
    return result;
  }

  HighRateAccelFilterConfig config_;
  std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> used_acceleration_history_;
  std::optional<Eigen::Vector3d> candidate_;
  std::size_t candidate_count_{0};
  bool sustained_confirmed_{false};
  std::size_t rejection_count_{0};
  std::size_t consecutive_rejections_{0};
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
  std::size_t accel_spike_rejection_count{0};
  std::size_t consecutive_accel_spike_rejections{0};
  double raw_acceleration_norm_mps2{0.0};
  double acceleration_deviation_mps2{0.0};
  bool accel_spike_rejected{false};
  bool accel_norm_limit_exceeded{false};
  bool accel_deviation_limit_exceeded{false};
  bool publish{false};
};

struct HighRateCorrectionDiagnostic
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};
  bool smoothing_enabled{false};
  double anchor_timestamp{0.0};
  double propagated_timestamp{0.0};
  double smoothing_elapsed_before{0.0};
  double smoothing_duration{0.0};
  Eigen::Vector3d propagated_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d anchor_posterior_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d posterior_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d output_velocity_before{Eigen::Vector3d::Zero()};
  Eigen::Vector3d output_velocity_after{Eigen::Vector3d::Zero()};
  Eigen::Vector3d correction_offset_velocity{Eigen::Vector3d::Zero()};
};

class HighRateOdomPropagator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HighRateOdomPropagator(
    HighRateOdomNoise noise = {}, double warn_anchor_age_s = 0.15,
    double max_anchor_age_s = 0.40, double history_duration_s = 5.0,
    HighRateAccelFilterConfig accel_filter = {},
    double correction_smoothing_s = 0.0)
  : noise_(noise),
    warn_anchor_age_s_(warn_anchor_age_s),
    max_anchor_age_s_(max_anchor_age_s),
    history_duration_s_(history_duration_s),
    accel_filter_(accel_filter),
    correction_smoothing_s_(std::max(0.0, correction_smoothing_s))
  {
    accel_filter_.window_size = std::max<std::size_t>(3, accel_filter_.window_size);
    if (accel_filter_.window_size % 2 == 0) {
      ++accel_filter_.window_size;
    }
    accel_filter_.min_samples = std::clamp<std::size_t>(
      accel_filter_.min_samples, 1, accel_filter_.window_size);
  }

  bool reset_from_lidar(const HighRateOdomState & corrected_state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::optional<HighRateOdomState> previous_output;
    const Eigen::Vector3d propagated_velocity = state_.velocity;
    const double propagated_timestamp = state_.timestamp;
    double smoothing_elapsed_before = 0.0;
    if (initialized_ && previous_input_) {
      previous_output = output_state_locked();
      smoothing_elapsed_before = smoothing_elapsed_locked();
    }
    if (!state_is_valid(corrected_state) ||
      (initialized_ && corrected_state.timestamp <= lidar_anchor_timestamp_) ||
      (!history_.empty() && corrected_state.timestamp > history_.back().timestamp))
    {
      return false;
    }

    // The current published state is the only valid starting point for a new
    // correction. Clear the old epoch before applying the posterior so an
    // unfinished correction can never be added to the next one.
    clear_correction_smoothing_locked();

    state_ = corrected_state;
    state_.rotation = orthonormalized(state_.rotation);
    state_.covariance = sanitized_covariance(state_.covariance);
    lidar_anchor_timestamp_ = corrected_state.timestamp;
    initialized_ = true;

    previous_input_ = input_at_locked(corrected_state.timestamp);
    if (previous_input_) {
      for (const auto & sample : history_) {
        if (sample.timestamp <= corrected_state.timestamp) {
          continue;
        }
        propagate_locked(sample);
      }
    }

    // LiDAR corrections are authoritative for the estimator, but publishing
    // the corrected state immediately would turn a centimetre-scale map
    // correction into a several-m/s 200 Hz velocity spike.  Keep the output
    // continuous and let it converge to the corrected trajectory smoothly.
    if (previous_output && correction_smoothing_s_ > 0.0) {
      correction_offset_position_ =
        previous_output->position - state_.position;
      correction_offset_velocity_ =
        previous_output->velocity - state_.velocity;
      smoothing_start_position_ = previous_output->position;
      smoothing_start_velocity_ = previous_output->velocity;
      correction_offset_start_timestamp_ = state_.timestamp;
    } else {
      clear_correction_smoothing_locked();
      smoothing_start_position_ = state_.position;
      smoothing_start_velocity_ = state_.velocity;
      correction_offset_start_timestamp_ = state_.timestamp;
    }

    HighRateCorrectionDiagnostic diagnostic;
    diagnostic.valid = true;
    diagnostic.smoothing_enabled = previous_output.has_value() &&
      correction_smoothing_s_ > 0.0;
    diagnostic.anchor_timestamp = corrected_state.timestamp;
    diagnostic.propagated_timestamp = propagated_timestamp;
    diagnostic.smoothing_elapsed_before = smoothing_elapsed_before;
    diagnostic.smoothing_duration = correction_smoothing_s_;
    diagnostic.propagated_velocity = propagated_velocity;
    diagnostic.anchor_posterior_velocity = corrected_state.velocity;
    diagnostic.posterior_velocity = state_.velocity;
    diagnostic.output_velocity_before = previous_output ?
      previous_output->velocity : propagated_velocity;
    diagnostic.output_velocity_after = output_state_locked().velocity;
    diagnostic.correction_offset_velocity = correction_offset_velocity_;
    last_correction_diagnostic_ = diagnostic;
    return true;
  }

  std::optional<HighRateCorrectionDiagnostic> take_last_correction_diagnostic()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto diagnostic = last_correction_diagnostic_;
    last_correction_diagnostic_.reset();
    return diagnostic;
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
        raw_acceleration_history_.clear();
        previous_input_.reset();
        initialized_ = false;
        accel_candidate_.reset();
        accel_candidate_count_ = 0;
        sustained_accel_confirmed_ = false;
        consecutive_accel_spike_rejections_ = 0;
        last_accel_spike_rejected_ = false;
      }
      return std::nullopt;
    }

    const HighRateImuSample filtered_sample = filter_acceleration_locked(sample);
    return add_filtered_imu_locked(filtered_sample);
  }

  // In production this is fed by HighRateImuPreprocessor so the main ESKF
  // and high-rate replay consume the exact same used acceleration.
  std::optional<HighRateOdomResult> add_filtered_imu(const HighRateImuSample & sample)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sample_is_valid(sample)) return std::nullopt;
    return add_filtered_imu_locked(sample);
  }

private:
  std::optional<HighRateOdomResult> add_filtered_imu_locked(const HighRateImuSample & sample)
  {
    if (!history_.empty() && sample.timestamp <= history_.back().timestamp) {
      if (sample.timestamp < history_.back().timestamp) {
        history_.clear();
        raw_acceleration_history_.clear();
        previous_input_.reset();
        initialized_ = false;
        accel_candidate_.reset();
        accel_candidate_count_ = 0;
        sustained_accel_confirmed_ = false;
        consecutive_accel_spike_rejections_ = 0;
        last_accel_spike_rejected_ = false;
      }
      return std::nullopt;
    }

    history_.push_back(sample);
    prune_history_locked(sample.timestamp);
    last_raw_acceleration_norm_mps2_ = sample.raw_acceleration_norm_mps2;
    last_acceleration_deviation_mps2_ = sample.acceleration_deviation_mps2;
    last_accel_spike_rejected_ = sample.accel_spike_rejected;
    last_accel_norm_limit_exceeded_ = sample.accel_norm_limit_exceeded;
    last_accel_deviation_limit_exceeded_ = sample.accel_deviation_limit_exceeded;
    accel_spike_rejection_count_ = sample.accel_spike_rejection_count;
    consecutive_accel_spike_rejections_ = sample.consecutive_accel_spike_rejections;
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

public:
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

  Eigen::Vector3d acceleration_median_locked() const
  {
    Eigen::Vector3d median = Eigen::Vector3d::Zero();
    std::vector<double> values;
    values.reserve(raw_acceleration_history_.size());
    for (int axis = 0; axis < 3; ++axis) {
      values.clear();
      for (const auto & acceleration : raw_acceleration_history_) {
        values.push_back(acceleration[axis]);
      }
      std::sort(values.begin(), values.end());
      const std::size_t middle = values.size() / 2;
      median[axis] = values[middle];
      if (values.size() % 2 == 0) {
        median[axis] = 0.5 * (median[axis] + values[middle - 1]);
      }
    }
    return median;
  }

  HighRateImuSample filter_acceleration_locked(const HighRateImuSample & sample)
  {
    HighRateImuSample filtered = sample;
    filtered.raw_acceleration = sample.acceleration;
    last_raw_acceleration_norm_mps2_ = sample.acceleration.norm();
    last_acceleration_deviation_mps2_ = 0.0;
    last_accel_norm_limit_exceeded_ = false;
    last_accel_deviation_limit_exceeded_ = false;
    last_accel_spike_rejected_ = false;

    if (!accel_filter_.enabled) {
      accel_candidate_.reset();
      accel_candidate_count_ = 0;
      sustained_accel_confirmed_ = false;
      consecutive_accel_spike_rejections_ = 0;
      return filtered;
    }

    const bool has_reference = !raw_acceleration_history_.empty();
    const Eigen::Vector3d reference = has_reference ?
      acceleration_median_locked() : sample.acceleration;
    last_acceleration_deviation_mps2_ = (sample.acceleration - reference).norm();
    last_accel_norm_limit_exceeded_ =
      accel_filter_.max_norm_mps2 > 0.0 &&
      last_raw_acceleration_norm_mps2_ > accel_filter_.max_norm_mps2;
    last_accel_deviation_limit_exceeded_ =
      raw_acceleration_history_.size() >= accel_filter_.min_samples &&
      accel_filter_.max_deviation_mps2 > 0.0 &&
      last_acceleration_deviation_mps2_ > accel_filter_.max_deviation_mps2;

    // A legitimate step in acceleration can remain away from the old median
    // for several samples.  Confirm repeated, norm-valid candidates before
    // allowing them through; norm-limit violations remain hard rejects.
    bool sustained_candidate = false;
    if (last_accel_norm_limit_exceeded_) {
      accel_candidate_.reset();
      accel_candidate_count_ = 0;
      sustained_accel_confirmed_ = false;
    } else if (last_accel_deviation_limit_exceeded_) {
      const bool candidate_matches = accel_candidate_.has_value() &&
        (sample.acceleration - *accel_candidate_).norm() <=
        accel_filter_.max_deviation_mps2;
      if (!candidate_matches) {
        accel_candidate_ = sample.acceleration;
        accel_candidate_count_ = 1;
        sustained_accel_confirmed_ = false;
      } else {
        ++accel_candidate_count_;
      }
      if (sustained_accel_confirmed_ ||
        accel_candidate_count_ > accel_filter_.min_samples)
      {
        sustained_accel_confirmed_ = true;
        sustained_candidate = true;
      }
    } else {
      accel_candidate_.reset();
      accel_candidate_count_ = 0;
      sustained_accel_confirmed_ = false;
    }

    last_accel_spike_rejected_ = has_reference &&
      (last_accel_norm_limit_exceeded_ ||
      (last_accel_deviation_limit_exceeded_ && !sustained_candidate));

    if (last_accel_spike_rejected_) {
      filtered.acceleration = reference;
      ++accel_spike_rejection_count_;
      ++consecutive_accel_spike_rejections_;
    } else {
      consecutive_accel_spike_rejections_ = 0;
    }

    // Keep the reference window on the acceleration actually used for
    // propagation.  Retaining rejected raw spikes lets a sustained burst
    // move the median and can cause valid samples to be rejected afterward.
    raw_acceleration_history_.push_back(filtered.acceleration);
    while (raw_acceleration_history_.size() > accel_filter_.window_size) {
      raw_acceleration_history_.pop_front();
    }
    filtered.accel_spike_rejection_count = accel_spike_rejection_count_;
    filtered.consecutive_accel_spike_rejections = consecutive_accel_spike_rejections_;
    filtered.raw_acceleration_norm_mps2 = last_raw_acceleration_norm_mps2_;
    filtered.acceleration_deviation_mps2 = last_acceleration_deviation_mps2_;
    filtered.accel_spike_rejected = last_accel_spike_rejected_;
    filtered.accel_norm_limit_exceeded = last_accel_norm_limit_exceeded_;
    filtered.accel_deviation_limit_exceeded = last_accel_deviation_limit_exceeded_;
    return filtered;
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

  HighRateOdomState output_state_locked() const
  {
    HighRateOdomState output = state_;
    if (correction_smoothing_s_ <= 0.0) {
      return output;
    }

    const double elapsed = output.timestamp - correction_offset_start_timestamp_;
    const double u = std::clamp(elapsed / correction_smoothing_s_, 0.0, 1.0);
    if (u >= 1.0) {
      return output;
    }

    // Cubic Hermite transition: preserve both pose and velocity at the first
    // sample, then reach the corrected trajectory with zero residual offset.
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double dh00 = (6.0 * u2 - 6.0 * u) / correction_smoothing_s_;
    const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    output.position += h00 * correction_offset_position_ +
      h10 * correction_smoothing_s_ * correction_offset_velocity_;
    output.velocity += dh00 * correction_offset_position_ +
      dh10 * correction_offset_velocity_;
    return output;
  }

  double smoothing_elapsed_locked() const
  {
    if (correction_smoothing_s_ <= 0.0) {
      return 0.0;
    }
    return std::max(0.0, state_.timestamp - correction_offset_start_timestamp_);
  }

  void clear_correction_smoothing_locked()
  {
    correction_offset_position_.setZero();
    correction_offset_velocity_.setZero();
    smoothing_start_position_ = state_.position;
    smoothing_start_velocity_ = state_.velocity;
    correction_offset_start_timestamp_ = state_.timestamp;
  }

  HighRateOdomResult make_result_locked() const
  {
    HighRateOdomResult result;
    result.state = output_state_locked();
    result.body_angular_velocity = last_body_angular_velocity_;
    result.lidar_anchor_age_s = state_.timestamp - lidar_anchor_timestamp_;
    result.accel_spike_rejection_count = accel_spike_rejection_count_;
    result.consecutive_accel_spike_rejections = consecutive_accel_spike_rejections_;
    result.raw_acceleration_norm_mps2 = last_raw_acceleration_norm_mps2_;
    result.acceleration_deviation_mps2 = last_acceleration_deviation_mps2_;
    result.accel_spike_rejected = last_accel_spike_rejected_;
    result.accel_norm_limit_exceeded = last_accel_norm_limit_exceeded_;
    result.accel_deviation_limit_exceeded = last_accel_deviation_limit_exceeded_;
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
  double history_duration_s_{5.0};
  double correction_smoothing_s_{0.0};
  HighRateAccelFilterConfig accel_filter_;
  mutable std::mutex mutex_;
  std::deque<HighRateImuSample, Eigen::aligned_allocator<HighRateImuSample>> history_;
  std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
    raw_acceleration_history_;
  HighRateOdomState state_;
  std::optional<HighRateImuSample> previous_input_;
  Eigen::Vector3d last_body_angular_velocity_{Eigen::Vector3d::Zero()};
  std::size_t accel_spike_rejection_count_{0};
  std::size_t consecutive_accel_spike_rejections_{0};
  double last_raw_acceleration_norm_mps2_{0.0};
  double last_acceleration_deviation_mps2_{0.0};
  bool last_accel_spike_rejected_{false};
  bool last_accel_norm_limit_exceeded_{false};
  bool last_accel_deviation_limit_exceeded_{false};
  std::optional<Eigen::Vector3d> accel_candidate_;
  std::size_t accel_candidate_count_{0};
  bool sustained_accel_confirmed_{false};
  double lidar_anchor_timestamp_{0.0};
  bool initialized_{false};
  Eigen::Vector3d correction_offset_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d correction_offset_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d smoothing_start_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d smoothing_start_velocity_{Eigen::Vector3d::Zero()};
  double correction_offset_start_timestamp_{0.0};
  std::optional<HighRateCorrectionDiagnostic> last_correction_diagnostic_;
};

}  // namespace fr_lio

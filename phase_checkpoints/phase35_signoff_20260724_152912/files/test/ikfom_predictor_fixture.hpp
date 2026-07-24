#ifndef FAST_LIO__TEST__IKFOM_PREDICTOR_FIXTURE_HPP_
#define FAST_LIO__TEST__IKFOM_PREDICTOR_FIXTURE_HPP_

#include "use-ikfom.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <stdexcept>

namespace fast_lio_test
{

// Exact value of G_m_s2 from FAST-LIO common_lib.h, kept local so this offline
// fixture does not acquire ROS message/runtime dependencies.
constexpr double kGravityMetersPerSecondSquared = 9.81;

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
using Covariance = Filter::cov;
using ProcessNoise = Filter::processnoisecovariance;

struct RawImuSample
{
  std::uint64_t sequence{};
  double timestamp{};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct NoiseDiagonals
{
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_bias{Eigen::Vector3d::Zero()};
};

struct PosteriorSnapshot
{
  double correction_timestamp{};
  state_ikfom state{};
  Covariance covariance{Covariance::Identity()};
  double mean_acceleration_norm{};
  NoiseDiagonals noise{};
};

struct PredictionResult
{
  state_ikfom state{};
  Covariance covariance{Covariance::Identity()};
  ProcessNoise process_noise{ProcessNoise::Zero()};
  input_ikfom normalized_input{};
  double dt{};
  std::uint64_t integrated_sequence{};
};

inline ProcessNoise make_process_noise(const NoiseDiagonals & noise)
{
  ProcessNoise q = ProcessNoise::Zero();
  q.block<3, 3>(0, 0).diagonal() = noise.gyro;
  q.block<3, 3>(3, 3).diagonal() = noise.acceleration;
  q.block<3, 3>(6, 6).diagonal() = noise.gyro_bias;
  q.block<3, 3>(9, 9).diagonal() = noise.acceleration_bias;
  return q;
}

inline input_ikfom make_normalized_midpoint_input(
  const RawImuSample & head,
  const RawImuSample & tail,
  double mean_acceleration_norm)
{
  if (!(mean_acceleration_norm > 0.0)) {
    throw std::invalid_argument("mean acceleration norm must be positive");
  }

  input_ikfom input;
  const Eigen::Vector3d acceleration_midpoint =
    0.5 * (head.acceleration + tail.acceleration);
  const Eigen::Vector3d gyro_midpoint =
    0.5 * (head.angular_velocity + tail.angular_velocity);
  input.acc =
    acceleration_midpoint * kGravityMetersPerSecondSquared / mean_acceleration_norm;
  input.gyro = gyro_midpoint;
  return input;
}

inline void initialize_process_model(Filter & filter)
{
  std::array<double, state_ikfom::DOF> limits{};
  limits.fill(1.0e-6);
  filter.init_dyn_runtime_share(
    get_f, df_dx, df_dw, 1, limits.data());
}

class IndependentIkfomPredictorFixture
{
public:
  explicit IndependentIkfomPredictorFixture(const PosteriorSnapshot & snapshot)
  : snapshot_(snapshot), filter_(snapshot.state, snapshot.covariance)
  {
    initialize_process_model(filter_);
  }

  PredictionResult predict_first_partial_interval(
    const RawImuSample * head,
    const RawImuSample * tail)
  {
    if (head == nullptr) {
      throw std::invalid_argument("first partial interval requires a predecessor");
    }
    if (tail == nullptr) {
      throw std::invalid_argument("first partial interval requires a new tail");
    }
    if (head->timestamp > snapshot_.correction_timestamp) {
      throw std::invalid_argument("head must not be newer than correction");
    }
    if (tail->timestamp <= snapshot_.correction_timestamp) {
      throw std::invalid_argument("tail must be newer than correction");
    }
    if (tail->sequence <= head->sequence) {
      throw std::invalid_argument("tail sequence must advance");
    }

    ProcessNoise q = make_process_noise(snapshot_.noise);
    const input_ikfom input = make_normalized_midpoint_input(
      *head, *tail, snapshot_.mean_acceleration_norm);
    double dt = tail->timestamp - snapshot_.correction_timestamp;
    filter_.predict(dt, q, input);

    PredictionResult result;
    result.state = filter_.get_x();
    result.covariance = filter_.get_P();
    result.process_noise = q;
    result.normalized_input = input;
    result.dt = dt;
    result.integrated_sequence = tail->sequence;
    return result;
  }

private:
  PosteriorSnapshot snapshot_;
  Filter filter_;
};

}  // namespace fast_lio_test

#endif  // FAST_LIO__TEST__IKFOM_PREDICTOR_FIXTURE_HPP_

#ifndef FAST_LIO__HIGH_RATE_EV_ODOMETRY_HPP_
#define FAST_LIO__HIGH_RATE_EV_ODOMETRY_HPP_

#include "high_rate_ev_propagator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>

namespace fast_lio
{

struct PropagatedOdometry
{
  std::uint64_t generation{0};
  std::uint64_t parent_generation{0};
  std::uint64_t validation_epoch{0};
  std::uint64_t imu_sequence{0};
  double timestamp{0.0};
  std::string frame_id{"camera_init"};
  std::string child_frame_id{"body"};
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_world_body{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  std::array<double, 36> pose_covariance{};
  std::array<double, 36> twist_covariance{};
};

class ExactImuStampIndex
{
public:
  explicit ExactImuStampIndex(std::size_t capacity = 1024);

  bool valid() const;
  bool record(double timestamp, std::int64_t nanoseconds);
  bool lookup(double timestamp, std::int64_t * nanoseconds) const;

private:
  struct Entry
  {
    double timestamp{0.0};
    std::int64_t nanoseconds{0};
  };

  std::size_t capacity_{0};
  std::deque<Entry> entries_;
};

class HighRateEvOdometryGate
{
public:
  struct Config
  {
    bool enabled{false};
    double publish_interval_s{0.025};
    double unknown_angular_velocity_variance{1.0e6};
    double covariance_symmetry_tolerance{1.0e-9};
    double covariance_psd_tolerance{1.0e-9};
  };

  HighRateEvOdometryGate();
  explicit HighRateEvOdometryGate(const Config & config);

  bool enabled() const;
  bool valid() const;
  bool prepare(
    const std::shared_ptr<const HighRateEvPropagator::Candidate> & candidate,
    PropagatedOdometry * output) const;
  bool commit(
    const std::shared_ptr<const HighRateEvPropagator::Candidate> & candidate);
  bool try_build(
    const std::shared_ptr<const HighRateEvPropagator::Candidate> & candidate,
    PropagatedOdometry * output);

private:
  bool validate_config() const;
  bool pair_is_publishable(const HighRateEvPropagator::Candidate & candidate) const;
  bool map_candidate(
    const HighRateEvPropagator::Candidate & candidate,
    PropagatedOdometry * output) const;

  Config config_;
  bool valid_{false};
  bool have_published_pair_{false};
  std::uint64_t last_published_sequence_{0};
  double last_published_timestamp_{0.0};
};

std::unique_ptr<HighRateEvOdometryGate> make_high_rate_ev_odometry_gate_if_enabled(
  const HighRateEvOdometryGate::Config & config);

}  // namespace fast_lio

#endif  // FAST_LIO__HIGH_RATE_EV_ODOMETRY_HPP_

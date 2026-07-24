#ifndef FAST_LIO__HIGH_RATE_EV_PROPAGATOR_HPP_
#define FAST_LIO__HIGH_RATE_EV_PROPAGATOR_HPP_

#include "use-ikfom.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <deque>
#ifdef FAST_LIO_PHASE2_TESTING
#include <functional>
#endif
#include <memory>
#include <mutex>
#include <string>

namespace fast_lio
{

class HighRateEvPropagator
{
public:
  using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
  using Covariance = Filter::cov;

  struct Config
  {
    bool enabled{false};
    double imu_timeout_s{0.050};
    double lidar_correction_timeout_s{0.30};
    double max_imu_dt_s{0.020};
    double future_tolerance_s{0.050};
    double ros_clock_jump_tolerance_s{0.050};
    double imu_buffer_span_s{1.0};
    std::size_t imu_buffer_max_count{512};
    double continuity_position_m{0.05};
    double continuity_attitude_rad{5.0 * 3.14159265358979323846 / 180.0};
    double continuity_velocity_mps{0.20};
    double recovery_healthy_s{2.0};
  };

  struct ImuSample
  {
    std::uint64_t sequence{0};
    double timestamp{0.0};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  };

  struct Noise
  {
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_bias{Eigen::Vector3d::Zero()};
  };

  struct PosteriorSnapshot
  {
    std::uint64_t generation{0};
    double correction_timestamp{0.0};
    state_ikfom state{};
    Covariance covariance{Covariance::Identity()};
    double mean_acceleration_norm{0.0};
    Noise noise{};
    input_ikfom last_normalized_input{};
  };

  enum class Health {kDisabled, kInitializing, kHealthy, kFault};
  enum class Phase {kEmpty, kReplaying, kReady, kActive, kCancelled, kRejected};
  enum class Fault {
    kNone, kInvalidInput, kTimestampOrder, kFutureTimestamp, kBufferOverflow,
    kMissingPredecessor, kInvalidPosterior, kImuGap, kReplayFailure,
    kContinuity, kLineage, kImuTimeout, kLidarTimeout, kClockJump,
    kEpochMismatch, kInvalidConfig
  };

  struct Diagnostics
  {
    Health health{Health::kDisabled};
    Fault fault{Fault::kNone};
    std::string detail{};
    std::uint64_t highest_generation{0};
    std::uint64_t latest_snapshot_generation{0};
    std::uint64_t active_generation{0};
    std::uint64_t active_parent_generation{0};
    std::uint64_t pending_generation{0};
    std::uint64_t pending_parent_generation{0};
    std::uint64_t validation_epoch{0};
    Phase active_phase{Phase::kEmpty};
    Phase pending_phase{Phase::kEmpty};
    std::uint64_t accepted_sequence{0};
    std::uint64_t active_imu_sequence{0};
    double active_timestamp{0.0};
    std::size_t buffer_count{0};
    std::size_t replay_count{0};
    std::uint64_t watermark_sequence{0};
    bool restart_required{false};
    bool recovery_waiting{false};
  };

  struct FixedDiagnosticsSnapshot
  {
    Health health{Health::kDisabled};
    Fault fault{Fault::kNone};
    std::uint64_t validation_epoch{0};
    std::uint64_t generation{0};
    std::uint64_t parent_generation{0};
    std::uint64_t last_candidate_sequence{0};
    double last_candidate_timestamp{0.0};
    double correction_age_s{-1.0};
    double imu_age_s{-1.0};
    double lidar_age_s{-1.0};
    std::size_t imu_buffer_count{0};
    double imu_buffer_span_s{0.0};
    std::uint64_t pending_watermark{0};
    std::uint64_t catch_up_count{0};
    bool recovery_waiting{false};
    bool recovery_started{false};
    double recovery_healthy_elapsed_s{0.0};
    double recovery_healthy_required_s{0.0};
    double recovery_progress{0.0};
  };

  struct Candidate
  {
    std::uint64_t generation{0};
    std::uint64_t parent_generation{0};
    std::uint64_t validation_epoch{0};
    std::uint64_t imu_sequence{0};
    double timestamp{0.0};
    state_ikfom state{};
    Covariance covariance{Covariance::Identity()};
  };

  HighRateEvPropagator();
  explicit HighRateEvPropagator(const Config & config);
  ~HighRateEvPropagator();
  bool ingest_imu(double timestamp, const Eigen::Vector3d & acceleration,
    const Eigen::Vector3d & angular_velocity, double ros_time, double steady_time);
  bool accept_posterior(
    const PosteriorSnapshot & snapshot, double ros_time, double steady_time);
  bool process(double ros_time, double steady_time);
  bool check_timeouts(double ros_time, double steady_time);
  void restart();
  Diagnostics diagnostics() const;
  FixedDiagnosticsSnapshot fixed_diagnostics(
    double ros_time, double steady_time) const;
  std::shared_ptr<const Candidate> candidate() const;

#ifdef FAST_LIO_PHASE2_TESTING
  void set_before_commit_hook(std::function<void(std::uint64_t)> hook);
#endif

private:
  struct Generation;
  bool replay(const PosteriorSnapshot &, const std::deque<ImuSample> &, Generation &);
  bool process_pending(
    const PosteriorSnapshot &, std::uint64_t parent_generation,
    double ros_time, double steady_time);
  bool advance(Generation &, const ImuSample &, const ImuSample &, double);
  bool catch_up_active(double ros_time, double steady_time);
  bool validate_config() const;
  bool validate_snapshot(const PosteriorSnapshot &) const;
  bool validate_generation(const Generation &) const;
  bool continuity_ok(const Generation &, const Generation &) const;
  bool observe_clock_locked(double ros_time, double steady_time);
  bool recovery_ready_locked(const Generation &, double steady_time) const;
  bool candidate_pair_advances_locked(const Candidate &) const;
  void store_candidate_locked(const std::shared_ptr<Candidate> &);
  void enter_fault(Fault, const std::string &, bool restart_required);
  void trim_buffer();

  Config config_;
  mutable std::mutex mutex_;
  std::deque<ImuSample> buffer_;
  std::shared_ptr<const Generation> active_;
  std::unique_ptr<PosteriorSnapshot> pending_snapshot_;
  std::uint64_t pending_parent_generation_{0};
  Diagnostics diagnostics_;
  bool restart_required_{false};
  bool recovery_required_{false};
  bool recovery_started_{false};
  double recovery_started_at_{0.0};
  bool clock_initialized_{false};
  double last_ros_time_{0.0};
  double last_steady_time_{0.0};
  bool have_imu_reference_{false};
  bool have_posterior_reference_{false};
  double last_imu_steady_time_{0.0};
  double last_posterior_steady_time_{0.0};
  bool have_last_accepted_timestamp_{false};
  double last_accepted_timestamp_{0.0};
  bool have_last_correction_timestamp_{false};
  double last_correction_timestamp_{0.0};
  bool have_candidate_guard_{false};
  std::uint64_t last_candidate_sequence_{0};
  double last_candidate_timestamp_{0.0};
  std::uint64_t validation_epoch_{0};
  std::shared_ptr<const Candidate> candidate_;
#ifdef FAST_LIO_PHASE2_TESTING
  std::function<void(std::uint64_t)> before_commit_hook_;
#endif
};

std::unique_ptr<HighRateEvPropagator> make_high_rate_ev_propagator_if_enabled(
  const HighRateEvPropagator::Config & config);

template<typename PropagatorT>
bool service_high_rate_ev_if_enabled(
  bool enabled, PropagatorT * propagator, double ros_time, double steady_time)
{
  if (!enabled || propagator == nullptr) {return false;}
  const bool timeout_ok = propagator->check_timeouts(ros_time, steady_time);
  const bool process_ok = propagator->process(ros_time, steady_time);
  return timeout_ok && process_ok;
}

}  // namespace fast_lio

#endif  // FAST_LIO__HIGH_RATE_EV_PROPAGATOR_HPP_

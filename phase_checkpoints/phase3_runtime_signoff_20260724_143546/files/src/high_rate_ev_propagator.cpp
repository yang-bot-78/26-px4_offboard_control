#include "high_rate_ev_propagator.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace fast_lio
{
namespace
{
bool finite_vector(const Eigen::Vector3d & value) {return value.allFinite();}

HighRateEvPropagator::Filter::processnoisecovariance make_q(
  const HighRateEvPropagator::Noise & noise)
{
  HighRateEvPropagator::Filter::processnoisecovariance q =
    HighRateEvPropagator::Filter::processnoisecovariance::Zero();
  q.block<3, 3>(0, 0).diagonal() = noise.gyro;
  q.block<3, 3>(3, 3).diagonal() = noise.acceleration;
  q.block<3, 3>(6, 6).diagonal() = noise.gyro_bias;
  q.block<3, 3>(9, 9).diagonal() = noise.acceleration_bias;
  return q;
}

void initialize_filter(HighRateEvPropagator::Filter & filter)
{
  std::array<double, state_ikfom::DOF> limits{};
  limits.fill(1.0e-6);
  filter.init_dyn_runtime_share(get_f, df_dx, df_dw, 1, limits.data());
}

bool finite_state(const state_ikfom & state)
{
  return state.pos.allFinite() && state.rot.coeffs().allFinite() &&
    state.offset_R_L_I.coeffs().allFinite() && state.offset_T_L_I.allFinite() &&
    state.vel.allFinite() && state.bg.allFinite() && state.ba.allFinite() &&
    state.grav.vec.allFinite();
}

bool interval_integrity_failure(
  const HighRateEvPropagator::ImuSample & head,
  const HighRateEvPropagator::ImuSample & tail,
  double from_time, double max_dt)
{
  const double raw_dt = tail.timestamp - head.timestamp;
  const double integration_dt = tail.timestamp - from_time;
  return !std::isfinite(from_time) || head.timestamp > from_time ||
    !(raw_dt > 0.0) || raw_dt > max_dt || !(integration_dt > 0.0) ||
    integration_dt > max_dt || tail.sequence <= head.sequence;
}

bool replay_integrity_failure(
  const HighRateEvPropagator::PosteriorSnapshot & snapshot,
  const std::deque<HighRateEvPropagator::ImuSample> & samples,
  double max_dt)
{
  auto tail = std::upper_bound(samples.begin(), samples.end(), snapshot.correction_timestamp,
    [](double stamp, const HighRateEvPropagator::ImuSample & sample) {
      return stamp < sample.timestamp;
    });
  if (tail == samples.begin() || tail == samples.end()) {return false;}
  auto head = std::prev(tail);
  double from_time = snapshot.correction_timestamp;
  for (; tail != samples.end(); ++head, ++tail) {
    if (interval_integrity_failure(*head, *tail, from_time, max_dt)) {return true;}
    from_time = tail->timestamp;
  }
  return false;
}

template<typename MatrixT>
bool finite_symmetric_psd(const MatrixT & matrix, double symmetry_tolerance,
  double eigenvalue_tolerance)
{
  if (!matrix.allFinite() ||
    (matrix - matrix.transpose()).cwiseAbs().maxCoeff() > symmetry_tolerance)
  {
    return false;
  }
  const MatrixT symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<MatrixT> solver(symmetric);
  return solver.info() == Eigen::Success && solver.eigenvalues().allFinite() &&
    solver.eigenvalues().minCoeff() >= -eigenvalue_tolerance;
}
}  // namespace

struct HighRateEvPropagator::Generation
{
  explicit Generation(const PosteriorSnapshot & snapshot, std::uint64_t parent)
  : generation(snapshot.generation), parent_generation(parent),
    correction_timestamp(snapshot.correction_timestamp), noise(snapshot.noise),
    mean_acceleration_norm(snapshot.mean_acceleration_norm),
    filter(snapshot.state, snapshot.covariance)
  {
    initialize_filter(filter);
  }

  std::uint64_t generation{0};
  std::uint64_t parent_generation{0};
  std::uint64_t validation_epoch{0};
  double correction_timestamp{0.0};
  Noise noise{};
  double mean_acceleration_norm{0.0};
  Filter filter;
  Phase phase{Phase::kReplaying};
  std::uint64_t last_sequence{0};
  double timestamp{0.0};
  ImuSample last_raw{};
  std::size_t replay_count{0};
};

HighRateEvPropagator::HighRateEvPropagator() : HighRateEvPropagator(Config()) {}

HighRateEvPropagator::HighRateEvPropagator(const Config & config) : config_(config)
{
  if (!config.enabled) {
    diagnostics_.health = Health::kDisabled;
  } else if (!validate_config()) {
    diagnostics_.health = Health::kFault;
    diagnostics_.fault = Fault::kInvalidConfig;
    diagnostics_.detail = "invalid Phase 2 configuration";
    diagnostics_.restart_required = true;
    restart_required_ = true;
  } else {
    diagnostics_.health = Health::kInitializing;
  }
}

HighRateEvPropagator::~HighRateEvPropagator() = default;

std::unique_ptr<HighRateEvPropagator> make_high_rate_ev_propagator_if_enabled(
  const HighRateEvPropagator::Config & config)
{
  if (!config.enabled) {return std::unique_ptr<HighRateEvPropagator>();}
  return std::unique_ptr<HighRateEvPropagator>(new HighRateEvPropagator(config));
}

bool HighRateEvPropagator::validate_config() const
{
  const Config defaults;
  const bool finite = std::isfinite(config_.imu_timeout_s) &&
    std::isfinite(config_.lidar_correction_timeout_s) &&
    std::isfinite(config_.max_imu_dt_s) && std::isfinite(config_.future_tolerance_s) &&
    std::isfinite(config_.ros_clock_jump_tolerance_s) &&
    std::isfinite(config_.imu_buffer_span_s) &&
    std::isfinite(config_.continuity_position_m) &&
    std::isfinite(config_.continuity_attitude_rad) &&
    std::isfinite(config_.continuity_velocity_mps) &&
    std::isfinite(config_.recovery_healthy_s);
  if (!finite || config_.max_imu_dt_s <= 0.0 || config_.imu_buffer_span_s <= 0.0) {
    return false;
  }
  const std::size_t minimum_buffer_count = static_cast<std::size_t>(
    std::ceil(config_.imu_buffer_span_s / config_.max_imu_dt_s)) + 1;
  return config_.imu_timeout_s > 0.0 &&
    config_.lidar_correction_timeout_s > 0.0 && config_.max_imu_dt_s > 0.0 &&
    config_.future_tolerance_s >= 0.0 && config_.ros_clock_jump_tolerance_s > 0.0 &&
    config_.imu_buffer_span_s >= config_.lidar_correction_timeout_s &&
    config_.imu_buffer_span_s > config_.max_imu_dt_s &&
    config_.imu_buffer_max_count >= minimum_buffer_count &&
    config_.continuity_position_m > 0.0 &&
    config_.continuity_position_m <= defaults.continuity_position_m &&
    config_.continuity_attitude_rad > 0.0 &&
    config_.continuity_attitude_rad <= defaults.continuity_attitude_rad &&
    config_.continuity_velocity_mps > 0.0 &&
    config_.continuity_velocity_mps <= defaults.continuity_velocity_mps &&
    config_.recovery_healthy_s >= 0.0;
}

bool HighRateEvPropagator::validate_snapshot(const PosteriorSnapshot & snapshot) const
{
  const auto q = make_q(snapshot.noise);
  if (snapshot.generation == 0 || !std::isfinite(snapshot.correction_timestamp) ||
    snapshot.correction_timestamp < 0.0 || !finite_state(snapshot.state) ||
    !std::isfinite(snapshot.mean_acceleration_norm) ||
    snapshot.mean_acceleration_norm <= 1.0e-9 ||
    !snapshot.last_normalized_input.acc.allFinite() ||
    !snapshot.last_normalized_input.gyro.allFinite() || !q.allFinite() ||
    (q.diagonal().array() < 0.0).any())
  {
    return false;
  }
  return finite_symmetric_psd(snapshot.covariance, 1.0e-9, 1.0e-10);
}

bool HighRateEvPropagator::validate_generation(const Generation & generation) const
{
  const state_ikfom state = generation.filter.get_x();
  const Covariance covariance = generation.filter.get_P();
  return finite_state(state) &&
    finite_symmetric_psd(covariance, 1.0e-8, 1.0e-9);
}

bool HighRateEvPropagator::advance(
  Generation & generation, const ImuSample & head, const ImuSample & tail, double from_time)
{
  const double raw_dt = tail.timestamp - head.timestamp;
  const double dt = tail.timestamp - from_time;
  if (!std::isfinite(from_time) || !std::isfinite(head.timestamp) ||
    !std::isfinite(tail.timestamp) || !finite_vector(head.acceleration) ||
    !finite_vector(head.angular_velocity) || !finite_vector(tail.acceleration) ||
    !finite_vector(tail.angular_velocity) || head.timestamp > from_time ||
    !(raw_dt > 0.0) || raw_dt > config_.max_imu_dt_s || !(dt > 0.0) ||
    dt > config_.max_imu_dt_s || tail.sequence <= head.sequence)
  {
    return false;
  }
  input_ikfom input;
  input.acc = 0.5 * (head.acceleration + tail.acceleration) * 9.81 /
    generation.mean_acceleration_norm;
  input.gyro = 0.5 * (head.angular_velocity + tail.angular_velocity);
  auto q = make_q(generation.noise);
  if (!input.acc.allFinite() || !input.gyro.allFinite() || !q.allFinite()) {return false;}
  double mutable_dt = dt;
  generation.filter.predict(mutable_dt, q, input);
  generation.timestamp = tail.timestamp;
  generation.last_sequence = tail.sequence;
  generation.last_raw = tail;
  ++generation.replay_count;
  return validate_generation(generation);
}

bool HighRateEvPropagator::replay(
  const PosteriorSnapshot & snapshot, const std::deque<ImuSample> & samples,
  Generation & generation)
{
  auto tail = std::upper_bound(samples.begin(), samples.end(), snapshot.correction_timestamp,
    [](double stamp, const ImuSample & sample) {return stamp < sample.timestamp;});
  if (tail == samples.begin() || tail == samples.end()) {return false;}
  auto head = std::prev(tail);
  double from_time = snapshot.correction_timestamp;
  for (; tail != samples.end(); ++head, ++tail) {
    if (!advance(generation, *head, *tail, from_time)) {return false;}
    from_time = tail->timestamp;
  }
  generation.phase = Phase::kReady;
  return true;
}

bool HighRateEvPropagator::continuity_ok(
  const Generation & old_generation, const Generation & new_generation) const
{
  if (old_generation.last_sequence != new_generation.last_sequence ||
    old_generation.timestamp != new_generation.timestamp)
  {
    return false;
  }
  const state_ikfom old_state = old_generation.filter.get_x();
  const state_ikfom new_state = new_generation.filter.get_x();
  Eigen::Vector3d rotation_delta;
  new_state.rot.boxminus(rotation_delta, old_state.rot);
  return (new_state.pos - old_state.pos).norm() <= config_.continuity_position_m &&
    rotation_delta.norm() <= config_.continuity_attitude_rad &&
    (new_state.vel - old_state.vel).norm() <= config_.continuity_velocity_mps;
}

bool HighRateEvPropagator::observe_clock_locked(double ros_time, double steady_time)
{
  if (!std::isfinite(ros_time) || !std::isfinite(steady_time)) {
    enter_fault(Fault::kInvalidInput, "non-finite clock sample", true);
    return false;
  }
  if (clock_initialized_) {
    const double ros_delta = ros_time - last_ros_time_;
    const double steady_delta = steady_time - last_steady_time_;
    if (steady_delta < 0.0 ||
      std::fabs(ros_delta - steady_delta) > config_.ros_clock_jump_tolerance_s)
    {
      enter_fault(Fault::kClockJump, "ROS/steady clock discontinuity", true);
      return false;
    }
  }
  clock_initialized_ = true;
  last_ros_time_ = ros_time;
  last_steady_time_ = steady_time;
  return true;
}

bool HighRateEvPropagator::candidate_pair_advances_locked(const Candidate & candidate) const
{
  return !have_candidate_guard_ ||
    (candidate.imu_sequence > last_candidate_sequence_ &&
    candidate.timestamp > last_candidate_timestamp_);
}

void HighRateEvPropagator::store_candidate_locked(
  const std::shared_ptr<Candidate> & candidate)
{
  if (!candidate_pair_advances_locked(*candidate)) {
    std::atomic_store_explicit(
      &candidate_, std::shared_ptr<const Candidate>(), std::memory_order_release);
    return;
  }
  have_candidate_guard_ = true;
  last_candidate_sequence_ = candidate->imu_sequence;
  last_candidate_timestamp_ = candidate->timestamp;
  std::atomic_store_explicit(
    &candidate_, std::shared_ptr<const Candidate>(candidate), std::memory_order_release);
}

bool HighRateEvPropagator::recovery_ready_locked(
  const Generation & generation, double steady_time) const
{
  if (!recovery_required_ || !recovery_started_ ||
    steady_time - recovery_started_at_ < config_.recovery_healthy_s ||
    !have_imu_reference_ || !have_posterior_reference_ ||
    steady_time - last_imu_steady_time_ > config_.imu_timeout_s ||
    steady_time - last_posterior_steady_time_ > config_.lidar_correction_timeout_s ||
    generation.generation != diagnostics_.latest_snapshot_generation)
  {
    return false;
  }
  return !have_candidate_guard_ ||
    (generation.last_sequence > last_candidate_sequence_ &&
    generation.timestamp > last_candidate_timestamp_);
}

bool HighRateEvPropagator::catch_up_active(double ros_time, double steady_time)
{
  (void)ros_time;
  for (;;) {
    std::shared_ptr<const Generation> base;
    std::vector<ImuSample> tails;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || (diagnostics_.health == Health::kFault && restart_required_)) {
        return false;
      }
      base = active_;
      for (const auto & sample : buffer_) {
        if (sample.sequence > base->last_sequence) {tails.push_back(sample);}
      }
    }
    if (tails.empty()) {return true;}
    std::shared_ptr<Generation> work(new Generation(*base));
    ImuSample head = work->last_raw;
    for (const auto & tail : tails) {
      const bool integrity_failure = interval_integrity_failure(
        head, tail, work->timestamp, config_.max_imu_dt_s);
      if (!advance(*work, head, tail, work->timestamp)) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_.get() == base.get()) {
          enter_fault(
            integrity_failure ? Fault::kImuGap : Fault::kReplayFailure,
            integrity_failure ? "active IMU interval integrity failure" :
            "active propagation failed", integrity_failure);
        }
        return false;
      }
      head = tail;
    }
    std::shared_ptr<Candidate> prepared(new Candidate());
    prepared->generation = work->generation;
    prepared->parent_generation = work->parent_generation;
    prepared->validation_epoch = work->validation_epoch;
    prepared->imu_sequence = work->last_sequence;
    prepared->timestamp = work->timestamp;
    prepared->state = work->filter.get_x();
    prepared->covariance = work->filter.get_P();
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_.get() != base.get()) {continue;}
    active_ = std::shared_ptr<const Generation>(work);
    diagnostics_.active_imu_sequence = active_->last_sequence;
    diagnostics_.active_timestamp = active_->timestamp;
    diagnostics_.watermark_sequence = buffer_.empty() ? 0 : buffer_.back().sequence;
    if (recovery_ready_locked(*active_, steady_time)) {
      recovery_required_ = false;
      recovery_started_ = false;
      diagnostics_.recovery_waiting = false;
      diagnostics_.health = Health::kHealthy;
      diagnostics_.fault = Fault::kNone;
      diagnostics_.detail.clear();
    }
    if (diagnostics_.health == Health::kHealthy) {store_candidate_locked(prepared);}
    if (buffer_.empty() || active_->last_sequence == buffer_.back().sequence) {return true;}
  }
}

bool HighRateEvPropagator::ingest_imu(
  double timestamp, const Eigen::Vector3d & acceleration,
  const Eigen::Vector3d & angular_velocity, double ros_time, double steady_time)
{
  if (!config_.enabled) {return false;}
  if (!std::isfinite(timestamp) || timestamp < 0.0 || !finite_vector(acceleration) ||
    !finite_vector(angular_velocity))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enter_fault(
      timestamp < 0.0 ? Fault::kEpochMismatch : Fault::kInvalidInput,
      "invalid IMU sample", timestamp < 0.0);
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (restart_required_ || !observe_clock_locked(ros_time, steady_time)) {return false;}
    if (timestamp > ros_time + config_.future_tolerance_s) {
      enter_fault(Fault::kFutureTimestamp, "IMU timestamp exceeds future tolerance", true);
      return false;
    }
    if (have_last_accepted_timestamp_ && timestamp <= last_accepted_timestamp_) {
      enter_fault(Fault::kTimestampOrder, "duplicate/backward IMU timestamp", true);
      return false;
    }
    if (ros_time - timestamp > config_.imu_timeout_s) {
      enter_fault(Fault::kImuTimeout, "IMU source age exceeds timeout", false);
      return false;
    }
    if (have_imu_reference_ &&
      steady_time - last_imu_steady_time_ > config_.imu_timeout_s)
    {
      enter_fault(Fault::kImuTimeout, "IMU timeout", false);
    }
    ImuSample sample;
    sample.sequence = ++diagnostics_.accepted_sequence;
    sample.timestamp = timestamp;
    sample.acceleration = acceleration;
    sample.angular_velocity = angular_velocity;
    buffer_.push_back(sample);
    have_last_accepted_timestamp_ = true;
    last_accepted_timestamp_ = timestamp;
    have_imu_reference_ = true;
    last_imu_steady_time_ = steady_time;
    if (buffer_.size() > config_.imu_buffer_max_count ||
      (buffer_.size() > 1 && buffer_.back().timestamp - buffer_.front().timestamp >
      config_.imu_buffer_span_s))
    {
      enter_fault(Fault::kBufferOverflow, "required IMU buffer bound exceeded", true);
      return false;
    }
    diagnostics_.buffer_count = buffer_.size();
    diagnostics_.watermark_sequence = buffer_.back().sequence;
  }
  return true;
}

bool HighRateEvPropagator::accept_posterior(
  const PosteriorSnapshot & snapshot, double ros_time, double steady_time)
{
  if (!config_.enabled) {return false;}
  if (std::isfinite(snapshot.correction_timestamp) &&
    snapshot.correction_timestamp < 0.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enter_fault(Fault::kEpochMismatch, "negative LiDAR correction time", true);
    return false;
  }
  if (!validate_snapshot(snapshot)) {
    std::lock_guard<std::mutex> lock(mutex_);
    enter_fault(Fault::kInvalidPosterior, "invalid posterior snapshot", false);
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (restart_required_ || !observe_clock_locked(ros_time, steady_time)) {return false;}
    if (snapshot.correction_timestamp > ros_time + config_.future_tolerance_s) {
      enter_fault(
        Fault::kFutureTimestamp, "LiDAR correction exceeds future tolerance", true);
      return false;
    }
    if (ros_time - snapshot.correction_timestamp > config_.lidar_correction_timeout_s) {
      enter_fault(Fault::kLidarTimeout, "LiDAR correction source age exceeds timeout", false);
      return false;
    }
    if (!buffer_.empty() &&
      std::fabs(snapshot.correction_timestamp - buffer_.back().timestamp) >
      config_.imu_buffer_span_s)
    {
      enter_fault(Fault::kEpochMismatch, "LiDAR/IMU timestamp epochs disagree", true);
      return false;
    }
    if (snapshot.generation <= diagnostics_.latest_snapshot_generation) {return false;}
    if (have_last_correction_timestamp_ &&
      snapshot.correction_timestamp <= last_correction_timestamp_)
    {
      enter_fault(Fault::kEpochMismatch, "non-increasing LiDAR correction time", true);
      return false;
    }
    diagnostics_.latest_snapshot_generation = snapshot.generation;
    diagnostics_.highest_generation = std::max(
      diagnostics_.highest_generation, snapshot.generation);
    pending_parent_generation_ = active_ ? active_->generation : 0;
    diagnostics_.pending_generation = snapshot.generation;
    diagnostics_.pending_parent_generation = pending_parent_generation_;
    diagnostics_.pending_phase = Phase::kReplaying;
    pending_snapshot_.reset(new PosteriorSnapshot(snapshot));
    have_last_correction_timestamp_ = true;
    last_correction_timestamp_ = snapshot.correction_timestamp;
    have_posterior_reference_ = true;
    last_posterior_steady_time_ = steady_time;
  }
  return true;
}

bool HighRateEvPropagator::process(double ros_time, double steady_time)
{
  if (!config_.enabled) {return false;}
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (restart_required_ || !observe_clock_locked(ros_time, steady_time)) {return false;}
  }
  catch_up_active(ros_time, steady_time);
  std::unique_ptr<PosteriorSnapshot> snapshot;
  std::uint64_t parent_generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (restart_required_) {return false;}
    if (pending_snapshot_) {
      snapshot.reset(new PosteriorSnapshot(*pending_snapshot_));
      parent_generation = pending_parent_generation_;
    }
  }
  if (!snapshot) {return true;}
  return process_pending(*snapshot, parent_generation, ros_time, steady_time);
}

bool HighRateEvPropagator::process_pending(
  const PosteriorSnapshot & snapshot, std::uint64_t parent_generation,
  double ros_time, double steady_time)
{
  std::deque<ImuSample> samples;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (diagnostics_.latest_snapshot_generation != snapshot.generation ||
      pending_parent_generation_ != parent_generation)
    {
      return false;
    }
    samples = buffer_;
  }

  std::shared_ptr<Generation> next(new Generation(snapshot, parent_generation));
  if (!replay(snapshot, samples, *next)) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (diagnostics_.latest_snapshot_generation == snapshot.generation) {
      const auto tail = std::upper_bound(samples.begin(), samples.end(),
        snapshot.correction_timestamp,
        [](double stamp, const ImuSample & sample) {return stamp < sample.timestamp;});
      if (tail == samples.begin()) {
        enter_fault(Fault::kMissingPredecessor, "posterior replay lacks a predecessor", true);
      } else if (tail != samples.end()) {
        const bool integrity_failure = replay_integrity_failure(
          snapshot, samples, config_.max_imu_dt_s);
        enter_fault(
          integrity_failure ? Fault::kImuGap : Fault::kReplayFailure,
          integrity_failure ? "posterior replay IMU interval integrity failure" :
          "posterior replay failed validation", integrity_failure);
      }
    }
    return false;
  }

  for (;;) {
    std::vector<ImuSample> tails;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (diagnostics_.latest_snapshot_generation != snapshot.generation ||
        pending_parent_generation_ != parent_generation)
      {
        return false;
      }
      for (const auto & sample : buffer_) {
        if (sample.sequence > next->last_sequence) {tails.push_back(sample);}
      }
    }
    ImuSample head = next->last_raw;
    for (const auto & tail : tails) {
      const bool integrity_failure = interval_integrity_failure(
        head, tail, next->timestamp, config_.max_imu_dt_s);
      if (!advance(*next, head, tail, next->timestamp)) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (diagnostics_.latest_snapshot_generation == snapshot.generation) {
          enter_fault(
            integrity_failure ? Fault::kImuGap : Fault::kReplayFailure,
            integrity_failure ? "pending IMU interval integrity failure" :
            "pending replay failed", integrity_failure);
        }
        return false;
      }
      head = tail;
    }

    std::shared_ptr<const Generation> active_for_check;
    std::uint64_t latest_watermark = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (diagnostics_.latest_snapshot_generation != snapshot.generation ||
        pending_parent_generation_ != parent_generation)
      {
        return false;
      }
      latest_watermark = buffer_.empty() ? 0 : buffer_.back().sequence;
      active_for_check = active_;
    }
    if (next->last_sequence != latest_watermark) {continue;}
    if ((active_for_check ? active_for_check->generation : 0) != parent_generation) {
      return false;
    }
    if (active_for_check && active_for_check->last_sequence != latest_watermark) {
      catch_up_active(ros_time, steady_time);
      continue;
    }
    const bool continuity_valid =
      !active_for_check || continuity_ok(*active_for_check, *next);
    std::shared_ptr<Candidate> prepared(new Candidate());
    prepared->generation = next->generation;
    prepared->parent_generation = next->parent_generation;
    prepared->imu_sequence = next->last_sequence;
    prepared->timestamp = next->timestamp;
    prepared->state = next->filter.get_x();
    prepared->covariance = next->filter.get_P();

#ifdef FAST_LIO_PHASE2_TESTING
    std::function<void(std::uint64_t)> hook;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hook = before_commit_hook_;
    }
    if (hook) {hook(snapshot.generation);}
#endif

    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t commit_watermark = buffer_.empty() ? 0 : buffer_.back().sequence;
    if (diagnostics_.latest_snapshot_generation != snapshot.generation ||
      pending_parent_generation_ != parent_generation ||
      commit_watermark != latest_watermark || active_.get() != active_for_check.get())
    {
      continue;
    }
    if (!continuity_valid) {
      enter_fault(Fault::kContinuity, "posterior continuity rejected", false);
      return false;
    }
    std::atomic_store_explicit(
      &candidate_, std::shared_ptr<const Candidate>(), std::memory_order_release);
    ++validation_epoch_;
    next->validation_epoch = validation_epoch_;
    next->phase = Phase::kActive;
    prepared->validation_epoch = validation_epoch_;
    active_ = std::shared_ptr<const Generation>(next);
    diagnostics_.active_generation = active_->generation;
    diagnostics_.active_parent_generation = active_->parent_generation;
    diagnostics_.active_phase = Phase::kActive;
    diagnostics_.pending_generation = 0;
    diagnostics_.pending_parent_generation = 0;
    diagnostics_.pending_phase = Phase::kEmpty;
    diagnostics_.active_imu_sequence = active_->last_sequence;
    diagnostics_.active_timestamp = active_->timestamp;
    diagnostics_.replay_count = active_->replay_count;
    diagnostics_.watermark_sequence = commit_watermark;
    diagnostics_.validation_epoch = validation_epoch_;
    if (diagnostics_.health == Health::kFault || recovery_required_) {
      recovery_required_ = true;
      if (!recovery_started_) {
        recovery_started_ = true;
        recovery_started_at_ = steady_time;
      }
      diagnostics_.recovery_waiting = true;
    } else {
      diagnostics_.health = Health::kHealthy;
      diagnostics_.fault = Fault::kNone;
      diagnostics_.detail.clear();
      store_candidate_locked(prepared);
    }
    trim_buffer();
    pending_snapshot_.reset();
    pending_parent_generation_ = 0;
    return true;
  }
}

bool HighRateEvPropagator::check_timeouts(double ros_time, double steady_time)
{
  if (!config_.enabled) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (restart_required_ || !observe_clock_locked(ros_time, steady_time)) {return false;}
  if (have_imu_reference_ &&
    steady_time - last_imu_steady_time_ > config_.imu_timeout_s)
  {
    enter_fault(Fault::kImuTimeout, "IMU timeout", false);
    return false;
  }
  if (have_posterior_reference_ &&
    steady_time - last_posterior_steady_time_ > config_.lidar_correction_timeout_s)
  {
    enter_fault(Fault::kLidarTimeout, "LiDAR correction timeout", false);
    return false;
  }
  return true;
}

void HighRateEvPropagator::trim_buffer()
{
  if (!active_) {return;}
  while (buffer_.size() > 1 && buffer_[1].timestamp <= active_->correction_timestamp) {
    buffer_.pop_front();
  }
  diagnostics_.buffer_count = buffer_.size();
}

void HighRateEvPropagator::enter_fault(
  Fault fault, const std::string & detail, bool restart_required)
{
  if (diagnostics_.health == Health::kFault && restart_required_) {return;}
  diagnostics_.health = Health::kFault;
  diagnostics_.fault = fault;
  diagnostics_.detail = detail;
  std::atomic_store_explicit(
    &candidate_, std::shared_ptr<const Candidate>(), std::memory_order_release);
  ++validation_epoch_;
  diagnostics_.validation_epoch = validation_epoch_;
  diagnostics_.active_phase = active_ ? Phase::kCancelled : Phase::kEmpty;
  diagnostics_.pending_phase = pending_snapshot_ ? Phase::kCancelled : Phase::kEmpty;
  active_.reset();
  pending_snapshot_.reset();
  pending_parent_generation_ = 0;
  buffer_.clear();
  diagnostics_.buffer_count = 0;
  diagnostics_.active_generation = 0;
  diagnostics_.active_parent_generation = 0;
  diagnostics_.pending_generation = 0;
  diagnostics_.pending_parent_generation = 0;
  diagnostics_.active_imu_sequence = 0;
  diagnostics_.active_timestamp = 0.0;
  diagnostics_.watermark_sequence = 0;
  diagnostics_.replay_count = 0;
  restart_required_ = restart_required_ || restart_required;
  recovery_required_ = !restart_required_;
  recovery_started_ = false;
  recovery_started_at_ = 0.0;
  have_imu_reference_ = false;
  have_posterior_reference_ = false;
  diagnostics_.restart_required = restart_required_;
  diagnostics_.recovery_waiting = recovery_required_;
}

void HighRateEvPropagator::restart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::atomic_store_explicit(
    &candidate_, std::shared_ptr<const Candidate>(), std::memory_order_release);
  ++validation_epoch_;
  active_.reset();
  pending_snapshot_.reset();
  pending_parent_generation_ = 0;
  buffer_.clear();
  restart_required_ = false;
  recovery_required_ = false;
  recovery_started_ = false;
  recovery_started_at_ = 0.0;
  clock_initialized_ = false;
  have_imu_reference_ = false;
  have_posterior_reference_ = false;
  diagnostics_.health = config_.enabled ? Health::kInitializing : Health::kDisabled;
  diagnostics_.fault = Fault::kNone;
  diagnostics_.detail.clear();
  if (config_.enabled && !validate_config()) {
    diagnostics_.health = Health::kFault;
    diagnostics_.fault = Fault::kInvalidConfig;
    diagnostics_.detail = "invalid Phase 2 configuration";
    diagnostics_.restart_required = true;
    restart_required_ = true;
  }
  diagnostics_.active_generation = 0;
  diagnostics_.active_parent_generation = 0;
  diagnostics_.pending_generation = 0;
  diagnostics_.pending_parent_generation = 0;
  diagnostics_.active_phase = Phase::kEmpty;
  diagnostics_.pending_phase = Phase::kEmpty;
  diagnostics_.buffer_count = 0;
  diagnostics_.active_imu_sequence = 0;
  diagnostics_.active_timestamp = 0.0;
  diagnostics_.watermark_sequence = 0;
  diagnostics_.replay_count = 0;
  diagnostics_.restart_required = restart_required_;
  diagnostics_.recovery_waiting = false;
  diagnostics_.validation_epoch = validation_epoch_;
}

HighRateEvPropagator::Diagnostics HighRateEvPropagator::diagnostics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return diagnostics_;
}

std::shared_ptr<const HighRateEvPropagator::Candidate>
HighRateEvPropagator::candidate() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto value = std::atomic_load_explicit(&candidate_, std::memory_order_acquire);
  if (!value || diagnostics_.health != Health::kHealthy || !active_ ||
    value->generation != active_->generation ||
    value->parent_generation != active_->parent_generation ||
    value->validation_epoch != active_->validation_epoch)
  {
    return std::shared_ptr<const Candidate>();
  }
  return value;
}

#ifdef FAST_LIO_PHASE2_TESTING
void HighRateEvPropagator::set_before_commit_hook(
  std::function<void(std::uint64_t)> hook)
{
  std::lock_guard<std::mutex> lock(mutex_);
  before_commit_hook_ = std::move(hook);
}
#endif
}  // namespace fast_lio

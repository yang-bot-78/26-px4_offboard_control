#include "high_rate_ev_propagator.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

namespace
{
using fast_lio::HighRateEvPropagator;
int failures = 0;

void expect(bool condition, const char * message)
{
  if (!condition) {std::cerr << "FAIL: " << message << '\n'; ++failures;}
}

HighRateEvPropagator::Config enabled_config()
{
  HighRateEvPropagator::Config config;
  config.enabled = true;
  return config;
}

void expect_fault(const HighRateEvPropagator & propagator,
  HighRateEvPropagator::Fault fault, bool restart_required, const char * message)
{
  const auto diagnostics = propagator.diagnostics();
  expect(diagnostics.health == HighRateEvPropagator::Health::kFault, message);
  expect(diagnostics.fault == fault, message);
  expect(diagnostics.restart_required == restart_required, message);
  expect(!propagator.candidate(), message);
}

HighRateEvPropagator::PosteriorSnapshot snapshot(
  std::uint64_t generation, double timestamp,
  const state_ikfom * state = nullptr,
  const HighRateEvPropagator::Covariance * covariance = nullptr)
{
  HighRateEvPropagator::PosteriorSnapshot value;
  value.generation = generation;
  value.correction_timestamp = timestamp;
  if (state) {value.state = *state;}
  if (covariance) {value.covariance = *covariance;}
  value.mean_acceleration_norm = 9.81;
  value.noise.gyro.setConstant(1.0e-4);
  value.noise.acceleration.setConstant(1.0e-4);
  value.noise.gyro_bias.setConstant(1.0e-6);
  value.noise.acceleration_bias.setConstant(1.0e-6);
  value.state.grav = S2(Eigen::Vector3d(0.0, 0.0, -9.809));
  return value;
}

bool add_imu(HighRateEvPropagator & propagator, double timestamp,
  double ros_time, double steady_time,
  const Eigen::Vector3d & gyro = Eigen::Vector3d::Zero())
{
  return propagator.ingest_imu(
    timestamp, Eigen::Vector3d(0.0, 0.0, 9.81), gyro, ros_time, steady_time);
}

bool add_imu_and_process(HighRateEvPropagator & propagator, double timestamp,
  double ros_time, double steady_time)
{
  return add_imu(propagator, timestamp, ros_time, steady_time) &&
    propagator.process(ros_time, steady_time);
}

bool accept_and_process(HighRateEvPropagator & propagator,
  const HighRateEvPropagator::PosteriorSnapshot & value,
  double ros_time, double steady_time)
{
  return propagator.accept_posterior(value, ros_time, steady_time) &&
    propagator.process(ros_time, steady_time);
}

struct TimeoutSpy
{
  bool check_timeouts(double ros_time, double steady_time)
  {
    ++calls;
    last_ros_time = ros_time;
    last_steady_time = steady_time;
    return result;
  }
  bool process(double ros_time, double steady_time)
  {
    ++process_calls;
    process_ros_time = ros_time;
    process_steady_time = steady_time;
    return process_result;
  }
  int calls{0};
  int process_calls{0};
  bool result{true};
  bool process_result{true};
  double last_ros_time{0.0};
  double last_steady_time{0.0};
  double process_ros_time{0.0};
  double process_steady_time{0.0};
};

void test_default_off_and_runtime_timeout_hook()
{
  HighRateEvPropagator::Config config;
  const auto propagator_ptr = fast_lio::make_high_rate_ev_propagator_if_enabled(config);
  expect(!propagator_ptr, "disabled factory must perform zero propagator construction");
  HighRateEvPropagator disabled;
  expect(disabled.diagnostics().health == HighRateEvPropagator::Health::kDisabled,
    "default object must remain disabled");
  expect(!add_imu(disabled, 1.0, 1.0, 1.0),
    "disabled component must not execute an IMU callback path");

  TimeoutSpy spy;
  expect(!fast_lio::service_high_rate_ev_if_enabled(
    false, &spy, 7.0, 11.0), "disabled runtime hook must be a no-op");
  expect(spy.calls == 0 && spy.process_calls == 0,
    "disabled runtime hook must make zero callbacks");
  expect(fast_lio::service_high_rate_ev_if_enabled(
    true, &spy, 7.0, 11.0), "enabled runtime hook must forward result");
  expect(spy.calls == 1 && spy.process_calls == 1 &&
    spy.last_ros_time == 7.0 && spy.last_steady_time == 11.0 &&
    spy.process_ros_time == 7.0 && spy.process_steady_time == 11.0,
    "runtime hook must forward independent ROS and steady times exactly");

  auto invalid = enabled_config();
  invalid.imu_buffer_max_count = 1;
  HighRateEvPropagator invalid_config(invalid);
  expect_fault(invalid_config, HighRateEvPropagator::Fault::kInvalidConfig, true,
    "invalid startup configuration must fail closed");
}

void test_complete_snapshot_and_timestamp_validation()
{
  {
    HighRateEvPropagator propagator(enabled_config());
    auto value = snapshot(1, 1.0);
    value.state.pos.x() = std::numeric_limits<double>::quiet_NaN();
    expect(!propagator.accept_posterior(value, 1.0, 1.0), "NaN state must reject");
    expect_fault(propagator, HighRateEvPropagator::Fault::kInvalidPosterior, false,
      "NaN state must latch automatic-recovery FAULT");
  }
  {
    HighRateEvPropagator propagator(enabled_config());
    auto value = snapshot(1, 2.0);
    value.covariance(0, 0) = -1.0;
    expect(!propagator.accept_posterior(value, 2.0, 2.0),
      "symmetric indefinite P must reject");
    expect_fault(propagator, HighRateEvPropagator::Fault::kInvalidPosterior, false,
      "indefinite P must latch FAULT");
  }
  {
    HighRateEvPropagator propagator(enabled_config());
    auto value = snapshot(1, 3.0);
    value.covariance(0, 1) = 1.0;
    expect(!propagator.accept_posterior(value, 3.0, 3.0), "asymmetric P must reject");
    expect_fault(propagator, HighRateEvPropagator::Fault::kInvalidPosterior, false,
      "asymmetric P must latch FAULT");
  }
  {
    HighRateEvPropagator propagator(enabled_config());
    auto value = snapshot(1, 4.0);
    value.noise.gyro.x() = -1.0;
    expect(!propagator.accept_posterior(value, 4.0, 4.0), "negative Q must reject");
    expect_fault(propagator, HighRateEvPropagator::Fault::kInvalidPosterior, false,
      "negative Q must latch FAULT");
  }
  {
    HighRateEvPropagator propagator(enabled_config());
    auto value = snapshot(1, 5.0);
    value.last_normalized_input.gyro.x() = std::numeric_limits<double>::infinity();
    expect(!propagator.accept_posterior(value, 5.0, 5.0),
      "non-finite normalized input must reject");
    expect_fault(propagator, HighRateEvPropagator::Fault::kInvalidPosterior, false,
      "non-finite normalized input must latch FAULT");
  }
  {
    HighRateEvPropagator propagator(enabled_config());
    auto value = snapshot(1, 6.10);
    expect(!propagator.accept_posterior(value, 6.0, 6.0),
      "future LiDAR correction must reject using independent now");
    expect_fault(propagator, HighRateEvPropagator::Fault::kFutureTimestamp, true,
      "future LiDAR correction must require restart");
  }
  {
    HighRateEvPropagator propagator(enabled_config());
    expect(!propagator.ingest_imu(
      std::numeric_limits<double>::quiet_NaN(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), 7.0, 7.0), "non-finite IMU time must reject");
    expect_fault(propagator, HighRateEvPropagator::Fault::kInvalidInput, false,
      "non-finite IMU time must latch FAULT");
  }
}

void test_partial_interval_generation_and_global_candidate_guard()
{
  HighRateEvPropagator propagator(enabled_config());
  expect(add_imu(propagator, 0.995, 1.000, 1.000), "partial predecessor must pass");
  expect(add_imu(propagator, 1.010, 1.015, 1.015), "partial tail must pass");
  expect(accept_and_process(propagator, snapshot(1, 1.000), 1.015, 1.015),
    "initial partial interval must activate");
  auto candidate = propagator.candidate();
  expect(candidate && candidate->generation == 1 && candidate->parent_generation == 0,
    "initial candidate lineage must be explicit");
  expect(candidate && candidate->imu_sequence == 2 && candidate->timestamp == 1.010,
    "initial candidate must integrate exactly the first new tail");
  const auto first_epoch = candidate ? candidate->validation_epoch : 0;

  expect(add_imu_and_process(propagator, 1.020, 1.025, 1.025),
    "active tail must pass");
  candidate = propagator.candidate();
  const state_ikfom corrected_state = candidate->state;
  const auto corrected_covariance = candidate->covariance;
  expect(accept_and_process(propagator,
    snapshot(2, 1.015, &corrected_state, &corrected_covariance), 1.025, 1.025),
    "generation 2 must switch at the common watermark");
  expect(!propagator.candidate(),
    "generation switch at an already exposed pair must not duplicate a candidate");
  const auto diagnostics = propagator.diagnostics();
  expect(diagnostics.active_generation == 2 && diagnostics.active_parent_generation == 1,
    "active generation must retain parent lineage");
  expect(diagnostics.active_imu_sequence == diagnostics.watermark_sequence,
    "active generation must commit only at the captured watermark");

  expect(add_imu_and_process(propagator, 1.030, 1.035, 1.035),
    "post-switch new tail must pass");
  candidate = propagator.candidate();
  expect(candidate && candidate->generation == 2 && candidate->parent_generation == 1,
    "post-switch candidate must belong to active lineage");
  expect(candidate && candidate->imu_sequence == 4 && candidate->timestamp == 1.030,
    "post-switch candidate must advance sequence and timestamp exactly once");
  expect(candidate && candidate->validation_epoch > first_epoch,
    "generation switch must advance validation epoch");

  HighRateEvPropagator exact_boundary(enabled_config());
  expect(add_imu(exact_boundary, 1.995, 2.000, 2.000), "boundary predecessor");
  expect(add_imu(exact_boundary, 2.000, 2.005, 2.005), "boundary context");
  expect(add_imu(exact_boundary, 2.006, 2.011, 2.011), "boundary new tail");
  expect(accept_and_process(exact_boundary, snapshot(1, 2.000), 2.011, 2.011),
    "tail at correction must become context");
  candidate = exact_boundary.candidate();
  expect(candidate && candidate->imu_sequence == 3 && candidate->timestamp == 2.006,
    "zero-duration boundary must not duplicate integration");
  expect(exact_boundary.diagnostics().replay_count == 1,
    "zero-duration boundary must execute exactly one prediction");
}

void test_deterministic_superseding_and_candidate_reader_concurrency()
{
  HighRateEvPropagator propagator(enabled_config());
  expect(add_imu(propagator, 2.995, 3.000, 3.000), "concurrency predecessor");
  expect(add_imu(propagator, 3.010, 3.015, 3.015), "concurrency first tail");
  expect(add_imu(propagator, 3.020, 3.025, 3.025), "concurrency watermark tail");
  expect(accept_and_process(propagator, snapshot(1, 3.000), 3.025, 3.025),
    "concurrency generation 1 must activate");
  const auto baseline = propagator.candidate();
  const state_ikfom state = baseline->state;
  const auto covariance = baseline->covariance;

  std::mutex barrier_mutex;
  std::condition_variable barrier_cv;
  bool generation_two_paused = false;
  bool release_generation_two = false;
  propagator.set_before_commit_hook([&](std::uint64_t generation) {
    if (generation != 2) {return;}
    std::unique_lock<std::mutex> lock(barrier_mutex);
    generation_two_paused = true;
    barrier_cv.notify_all();
    barrier_cv.wait(lock, [&]() {return release_generation_two;});
  });

  expect(propagator.accept_posterior(
    snapshot(2, 3.010, &state, &covariance), 3.025, 3.025),
    "generation 2 must enter pending before deterministic pause");
  bool generation_two_result = true;
  std::thread stale_completion([&]() {
    generation_two_result = propagator.process(3.025, 3.025);
  });
  {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    barrier_cv.wait(lock, [&]() {return generation_two_paused;});
  }
  expect(propagator.accept_posterior(
    snapshot(3, 3.015, &state, &covariance), 3.025, 3.025),
    "newer generation must supersede a paused completion");
  expect(propagator.process(3.025, 3.025),
    "newer generation must process while the stale completion is paused");
  {
    std::lock_guard<std::mutex> lock(barrier_mutex);
    release_generation_two = true;
  }
  barrier_cv.notify_all();
  stale_completion.join();
  expect(!generation_two_result, "stale generation completion must be rejected");
  expect(propagator.diagnostics().active_generation == 3,
    "stale completion must not replace the newest active generation");
  expect(!propagator.candidate(),
    "same-watermark superseding switch must not duplicate candidate");
  propagator.set_before_commit_hook({});

  std::atomic<bool> stop_reader{false};
  std::atomic<bool> reader_failed{false};
  std::thread reader([&]() {
    std::uint64_t last_sequence = 0;
    while (!stop_reader.load()) {
      const auto value = propagator.candidate();
      if (value) {
        if (value->generation != 3 || value->parent_generation != 1 ||
          value->imu_sequence < last_sequence)
        {
          reader_failed.store(true);
        }
        last_sequence = value->imu_sequence;
      }
    }
  });
  double timestamp = 3.020;
  for (int index = 0; index < 20; ++index) {
    timestamp += 0.005;
    expect(add_imu_and_process(propagator, timestamp, 3.030 + index * 0.005,
      3.030 + index * 0.005), "concurrent writer IMU must pass");
  }
  stop_reader.store(true);
  reader.join();
  expect(!reader_failed.load(),
    "candidate reader must never observe stale lineage or sequence rollback");
  const auto final_candidate = propagator.candidate();
  expect(final_candidate && final_candidate->imu_sequence == 23,
    "independent sequence oracle must account for every accepted tail exactly once");

  const state_ikfom pending_state = final_candidate ? final_candidate->state : state_ikfom();
  const auto pending_covariance = final_candidate ?
    final_candidate->covariance : HighRateEvPropagator::Covariance::Identity();
  {
    std::lock_guard<std::mutex> lock(barrier_mutex);
    generation_two_paused = false;
    release_generation_two = false;
  }
  propagator.set_before_commit_hook([&](std::uint64_t generation) {
    if (generation != 4) {return;}
    std::unique_lock<std::mutex> lock(barrier_mutex);
    generation_two_paused = true;
    barrier_cv.notify_all();
    barrier_cv.wait(lock, [&]() {return release_generation_two;});
  });
  expect(propagator.accept_posterior(
    snapshot(4, 3.110, &pending_state, &pending_covariance), 3.125, 3.125),
    "generation 4 must enter pending before tail-arrival pause");
  bool paused_pending_result = true;
  std::thread paused_pending([&]() {
    paused_pending_result = propagator.process(3.125, 3.125);
  });
  {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    barrier_cv.wait(lock, [&]() {return generation_two_paused;});
  }
  propagator.set_before_commit_hook({});
  expect(add_imu(propagator, 3.125, 3.130, 3.130),
    "tail arriving during pending READY must be accepted");
  expect(propagator.process(3.130, 3.130),
    "existing timer worker must catch active and pending up to the new tail");
  {
    std::lock_guard<std::mutex> lock(barrier_mutex);
    release_generation_two = true;
  }
  barrier_cv.notify_all();
  paused_pending.join();
  expect(!paused_pending_result,
    "older same-generation completion must lose to the tail catch-up retry");
  const auto caught_up = propagator.diagnostics();
  expect(caught_up.active_generation == 4 && caught_up.active_imu_sequence == 24 &&
    caught_up.active_imu_sequence == caught_up.watermark_sequence,
    "pending generation must catch the new tail with no omission before commit");
  expect(!propagator.candidate(),
    "pending catch-up switch must not duplicate the already exposed pair");
  expect(add_imu_and_process(propagator, 3.130, 3.135, 3.135),
    "first post-catch-up tail must be accepted");
  const auto post_catch_up = propagator.candidate();
  expect(post_catch_up && post_catch_up->generation == 4 &&
    post_catch_up->parent_generation == 3 && post_catch_up->imu_sequence == 25,
    "first post-catch-up candidate must advance exactly once on the new lineage");
}

void test_runtime_imu_and_lidar_timeout_invalidation()
{
  auto imu_config = enabled_config();
  imu_config.imu_timeout_s = 0.05;
  HighRateEvPropagator imu_timeout(imu_config);
  expect(add_imu(imu_timeout, 7.995, 8.000, 8.000), "IMU timeout predecessor");
  expect(add_imu(imu_timeout, 8.010, 8.015, 8.015), "IMU timeout tail");
  expect(accept_and_process(imu_timeout, snapshot(1, 8.000), 8.015, 8.015),
    "IMU timeout active seed");
  expect(imu_timeout.candidate() != nullptr, "IMU timeout seed candidate");
  expect(!fast_lio::service_high_rate_ev_if_enabled(
    true, &imu_timeout, 8.070, 8.070), "runtime hook must fire IMU timeout");
  expect_fault(imu_timeout, HighRateEvPropagator::Fault::kImuTimeout, false,
    "IMU timeout must atomically invalidate candidate");

  auto lidar_config = enabled_config();
  lidar_config.imu_timeout_s = 1.0;
  lidar_config.lidar_correction_timeout_s = 0.10;
  HighRateEvPropagator lidar_timeout(lidar_config);
  expect(add_imu(lidar_timeout, 8.995, 9.000, 9.000), "LiDAR timeout predecessor");
  expect(add_imu(lidar_timeout, 9.010, 9.015, 9.015), "LiDAR timeout tail");
  expect(accept_and_process(lidar_timeout, snapshot(1, 9.000), 9.015, 9.015),
    "LiDAR timeout active seed");
  expect(add_imu_and_process(lidar_timeout, 9.020, 9.080, 9.080),
    "fresh IMU must keep active candidate advancing before LiDAR timeout");
  expect(!fast_lio::service_high_rate_ev_if_enabled(
    true, &lidar_timeout, 9.120, 9.120), "runtime hook must fire LiDAR timeout");
  expect_fault(lidar_timeout, HighRateEvPropagator::Fault::kLidarTimeout, false,
    "LiDAR timeout must latch despite continuing IMU");
}

void test_clock_and_epoch_fault_matrix()
{
  {
    HighRateEvPropagator positive_jump(enabled_config());
    expect(add_imu(positive_jump, 1.0, 1.0, 1.0), "positive jump seed");
    expect(!positive_jump.check_timeouts(2.0, 1.01), "positive ROS jump must reject");
    expect_fault(positive_jump, HighRateEvPropagator::Fault::kClockJump, true,
      "positive ROS jump must require restart");
  }
  {
    HighRateEvPropagator negative_jump(enabled_config());
    expect(add_imu(negative_jump, 2.0, 2.0, 2.0), "negative jump seed");
    expect(!negative_jump.check_timeouts(1.0, 2.01), "negative ROS jump must reject");
    expect_fault(negative_jump, HighRateEvPropagator::Fault::kClockJump, true,
      "negative ROS jump must require restart");
  }
  {
    HighRateEvPropagator future(enabled_config());
    expect(!add_imu(future, 3.10, 3.0, 3.0), "future IMU must reject");
    expect_fault(future, HighRateEvPropagator::Fault::kFutureTimestamp, true,
      "future IMU must require restart");
  }
  {
    HighRateEvPropagator duplicate(enabled_config());
    expect(add_imu(duplicate, 4.0, 4.0, 4.0), "duplicate seed");
    expect(!add_imu(duplicate, 4.0, 4.01, 4.01), "duplicate IMU must reject");
    expect_fault(duplicate, HighRateEvPropagator::Fault::kTimestampOrder, true,
      "duplicate IMU must require restart");
    const auto sequence = duplicate.diagnostics().accepted_sequence;
    duplicate.restart();
    expect(duplicate.diagnostics().accepted_sequence == sequence,
      "accepted sequence must remain monotonic across restart");
    expect(!add_imu(duplicate, 3.9, 4.02, 4.02),
      "lower sensor epoch must remain rejected after restart");
    expect_fault(duplicate, HighRateEvPropagator::Fault::kTimestampOrder, true,
      "lower sensor epoch must not reset node-lifetime ordering");
  }
  {
    HighRateEvPropagator correction_epoch(enabled_config());
    expect(add_imu(correction_epoch, 4.995, 5.000, 5.000), "correction epoch head");
    expect(add_imu(correction_epoch, 5.010, 5.015, 5.015), "correction epoch tail");
    expect(accept_and_process(correction_epoch, snapshot(1, 5.000), 5.015, 5.015),
      "correction epoch seed");
    expect(!correction_epoch.accept_posterior(snapshot(2, 5.000), 5.020, 5.020),
      "non-increasing correction time must reject");
    expect_fault(correction_epoch, HighRateEvPropagator::Fault::kEpochMismatch, true,
      "non-increasing correction time must require restart");
  }
  {
    HighRateEvPropagator stale_imu(enabled_config());
    expect(!add_imu(stale_imu, 6.0, 6.10, 6.10),
      "IMU source age beyond timeout must reject");
    expect_fault(stale_imu, HighRateEvPropagator::Fault::kImuTimeout, false,
      "stale IMU source age must use automatic recovery authority");
  }
  {
    HighRateEvPropagator stale_lidar(enabled_config());
    expect(add_imu(stale_lidar, 6.995, 7.000, 7.000), "stale LiDAR head");
    expect(add_imu(stale_lidar, 7.010, 7.015, 7.015), "stale LiDAR tail");
    expect(!stale_lidar.accept_posterior(snapshot(1, 7.000), 7.400, 7.400),
      "LiDAR source age beyond timeout must reject");
    expect_fault(stale_lidar, HighRateEvPropagator::Fault::kLidarTimeout, false,
      "stale LiDAR source age must use automatic recovery authority");
  }
  {
    HighRateEvPropagator mismatched_epoch(enabled_config());
    expect(add_imu(mismatched_epoch, 8.000, 8.000, 8.000), "epoch mismatch IMU");
    expect(!mismatched_epoch.accept_posterior(snapshot(1, 10.000), 10.000, 10.000),
      "LiDAR/IMU epochs separated beyond buffer span must reject");
    expect_fault(mismatched_epoch, HighRateEvPropagator::Fault::kEpochMismatch, true,
      "LiDAR/IMU epoch mismatch must require restart");
  }
  {
    HighRateEvPropagator negative_correction(enabled_config());
    expect(!negative_correction.accept_posterior(snapshot(1, -0.1), 0.0, 0.0),
      "negative correction timestamp must reject");
    expect_fault(negative_correction, HighRateEvPropagator::Fault::kEpochMismatch, true,
      "negative correction timestamp must require restart");
  }
}

void test_automatic_recovery_requires_full_healthy_matrix()
{
  auto config = enabled_config();
  config.imu_timeout_s = 0.06;
  config.lidar_correction_timeout_s = 0.10;
  config.max_imu_dt_s = 0.03;
  config.recovery_healthy_s = 0.05;
  HighRateEvPropagator propagator(config);
  expect(add_imu(propagator, 9.995, 10.000, 10.000), "recovery initial head");
  expect(add_imu(propagator, 10.010, 10.015, 10.015), "recovery initial tail");
  expect(accept_and_process(propagator, snapshot(1, 10.000), 10.015, 10.015),
    "recovery initial generation");
  const auto pre_fault = propagator.candidate();
  expect(!propagator.check_timeouts(10.080, 10.080), "recovery IMU timeout trigger");
  expect_fault(propagator, HighRateEvPropagator::Fault::kImuTimeout, false,
    "IMU timeout must select automatic posterior recovery");

  expect(add_imu(propagator, 10.090, 10.090, 10.090), "recovery clean head");
  expect(add_imu(propagator, 10.105, 10.105, 10.105), "recovery clean tail");
  expect(accept_and_process(propagator, snapshot(2, 10.095), 10.105, 10.105),
    "new posterior must build recovery lineage");
  expect(propagator.diagnostics().recovery_waiting && !propagator.candidate(),
    "recovery must suppress candidate before healthy interval");
  expect(add_imu_and_process(propagator, 10.120, 10.120, 10.120),
    "early recovery IMU");
  expect(propagator.diagnostics().health == HighRateEvPropagator::Health::kFault,
    "elapsed time below recovery_healthy_s must remain FAULT");
  expect(add_imu_and_process(propagator, 10.135, 10.135, 10.135),
    "recovery correction tail");
  expect(accept_and_process(propagator, snapshot(3, 10.125), 10.135, 10.135),
    "fresh LiDAR correction must preserve recovery continuity");
  expect(add_imu_and_process(propagator, 10.150, 10.160, 10.160),
    "post-interval recovery IMU must pass");
  expect(propagator.diagnostics().health == HighRateEvPropagator::Health::kHealthy,
    "full fresh IMU/LiDAR/state matrix must restore HEALTHY");
  const auto recovered = propagator.candidate();
  expect(recovered && pre_fault &&
    recovered->imu_sequence > pre_fault->imu_sequence &&
    recovered->timestamp > pre_fault->timestamp && recovered->generation == 3,
    "first recovered candidate must exceed preserved global pair on new lineage");

  auto stale_config = config;
  stale_config.recovery_healthy_s = 0.20;
  HighRateEvPropagator stale_lidar(stale_config);
  expect(add_imu(stale_lidar, 19.995, 20.000, 20.000), "stale recovery head");
  expect(add_imu(stale_lidar, 20.010, 20.015, 20.015), "stale recovery tail");
  expect(accept_and_process(stale_lidar, snapshot(1, 20.000), 20.015, 20.015),
    "stale recovery initial generation");
  expect(!stale_lidar.check_timeouts(20.080, 20.080), "stale recovery timeout");
  expect(add_imu(stale_lidar, 20.090, 20.090, 20.090), "stale recovery clean head");
  expect(add_imu(stale_lidar, 20.105, 20.105, 20.105), "stale recovery clean tail");
  expect(accept_and_process(stale_lidar, snapshot(2, 20.095), 20.105, 20.105),
    "stale recovery posterior");
  expect(add_imu_and_process(stale_lidar, 20.120, 20.120, 20.120),
    "stale recovery IMU one");
  expect(add_imu_and_process(stale_lidar, 20.135, 20.135, 20.135),
    "stale recovery IMU two");
  expect(add_imu_and_process(stale_lidar, 20.150, 20.150, 20.150),
    "stale recovery IMU three");
  expect(add_imu_and_process(stale_lidar, 20.170, 20.170, 20.170),
    "stale recovery IMU four");
  expect(add_imu_and_process(stale_lidar, 20.190, 20.190, 20.190),
    "stale recovery IMU five");
  expect(!stale_lidar.check_timeouts(20.210, 20.210),
    "missing LiDAR during recovery must relatch timeout before HEALTHY");
  expect_fault(stale_lidar, HighRateEvPropagator::Fault::kLidarTimeout, false,
    "stale LiDAR must prevent elapsed-time-only recovery");
}

void test_hard_and_automatic_fault_recovery_authority_matrix()
{
  {
    HighRateEvPropagator missing(enabled_config());
    expect(add_imu(missing, 30.010, 30.010, 30.010), "missing predecessor tail");
    expect(missing.accept_posterior(snapshot(1, 30.000), 30.010, 30.010),
      "missing predecessor snapshot must enqueue before worker validation");
    expect(!missing.process(30.010, 30.010),
      "worker must reject a pending replay with no predecessor");
    expect_fault(missing, HighRateEvPropagator::Fault::kMissingPredecessor, true,
      "required predecessor loss must require restart");
  }
  {
    auto config = enabled_config();
    config.imu_buffer_span_s = 0.30;
    config.imu_buffer_max_count = 32;
    HighRateEvPropagator overflow(config);
    for (int index = 0; index < 32; ++index) {
      const double timestamp = 31.000 + 0.005 * index;
      expect(add_imu(overflow, timestamp, timestamp, timestamp),
        "bounded buffer sample must pass before hard count");
    }
    expect(!add_imu(overflow, 31.160, 31.160, 31.160),
      "hard-count overflow sample must reject");
    expect_fault(overflow, HighRateEvPropagator::Fault::kBufferOverflow, true,
      "required buffer loss must require restart");
  }
  {
    HighRateEvPropagator invalid(enabled_config());
    expect(!invalid.ingest_imu(32.0,
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
      Eigen::Vector3d::Zero(), 32.0, 32.0), "non-finite payload must reject");
    expect_fault(invalid, HighRateEvPropagator::Fault::kInvalidInput, false,
      "single non-finite payload must use automatic posterior recovery authority");
  }
  {
    HighRateEvPropagator gap(enabled_config());
    expect(add_imu(gap, 32.995, 33.000, 33.000), "gap predecessor");
    expect(add_imu(gap, 33.025, 33.030, 33.030), "gap tail ingress");
    expect(gap.accept_posterior(snapshot(1, 33.000), 33.030, 33.030),
      "gap snapshot must enqueue before worker interval validation");
    expect(!gap.process(33.030, 33.030), "unbridgeable replay gap must reject");
    expect_fault(gap, HighRateEvPropagator::Fault::kImuGap, true,
      "required replay interval loss must require restart");
  }
}
}  // namespace

int main()
{
  test_default_off_and_runtime_timeout_hook();
  test_complete_snapshot_and_timestamp_validation();
  test_partial_interval_generation_and_global_candidate_guard();
  test_deterministic_superseding_and_candidate_reader_concurrency();
  test_runtime_imu_and_lidar_timeout_invalidation();
  test_clock_and_epoch_fault_matrix();
  test_automatic_recovery_requires_full_healthy_matrix();
  test_hard_and_automatic_fault_recovery_authority_matrix();
  if (failures != 0) {
    std::cerr << failures << " Phase 2 assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "Eight independent Phase 2 concurrency/timeout/epoch/recovery tests passed\n";
  return EXIT_SUCCESS;
}

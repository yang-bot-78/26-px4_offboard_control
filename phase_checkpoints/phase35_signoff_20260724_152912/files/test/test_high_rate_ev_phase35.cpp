#include "high_rate_ev_diagnostics.hpp"
#include "high_rate_ev_odometry.hpp"

#include <Eigen/Core>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
using fast_lio::HighRateEvDiagnosticInput;
using fast_lio::HighRateEvOdometryGate;
using fast_lio::HighRateEvPropagator;
using fast_lio::Phase3DiagnosticsStore;
using Candidate = HighRateEvPropagator::Candidate;

int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

std::string value(
  const diagnostic_msgs::msg::DiagnosticArray & message,
  const std::string & key)
{
  if (message.status.size() != 1) {return std::string();}
  for (const auto & item : message.status.front().values) {
    if (item.key == key) {return item.value;}
  }
  return std::string();
}

diagnostic_msgs::msg::DiagnosticArray make_message(
  const HighRateEvDiagnosticInput & input)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456;
  return fast_lio::make_high_rate_ev_diagnostic_array(input, stamp);
}

HighRateEvPropagator::PosteriorSnapshot recovery_snapshot(
  std::uint64_t generation, double timestamp)
{
  HighRateEvPropagator::PosteriorSnapshot snapshot;
  snapshot.generation = generation;
  snapshot.correction_timestamp = timestamp;
  snapshot.mean_acceleration_norm = 9.81;
  snapshot.noise.gyro.setConstant(1.0e-4);
  snapshot.noise.acceleration.setConstant(1.0e-4);
  snapshot.noise.gyro_bias.setConstant(1.0e-6);
  snapshot.noise.acceleration_bias.setConstant(1.0e-6);
  snapshot.state.grav = S2(Eigen::Vector3d(0.0, 0.0, -9.809));
  return snapshot;
}

bool recovery_imu(
  HighRateEvPropagator & propagator, double timestamp, double now)
{
  return propagator.ingest_imu(
    timestamp, Eigen::Vector3d(0.0, 0.0, 9.81),
    Eigen::Vector3d::Zero(), now, now);
}

void test_default_off_and_required_keys()
{
  expect(!fast_lio::high_rate_ev_diagnostics_enabled(false),
    "default-off must not enable the diagnostic publisher");
  expect(fast_lio::high_rate_ev_diagnostics_enabled(true),
    "Phase 3 enable must enable the diagnostic publisher");

  HighRateEvDiagnosticInput input;
  input.phase2_enabled = true;
  input.phase3_enabled = true;
  input.propagator.health = HighRateEvPropagator::Health::kHealthy;
  input.propagator.generation = 8;
  input.propagator.parent_generation = 7;
  input.propagator.validation_epoch = 9;
  input.propagator.pending_watermark = 101;
  input.propagator.catch_up_count = 3;
  input.publisher.last_candidate_sequence = 100;
  input.publisher.last_candidate_timestamp_ns = 123456789;
  input.publisher.last_published_sequence = 99;
  input.publisher.last_published_timestamp_ns = 123450000;
  input.publisher.timeout_count = 2;
  const auto message = make_message(input);
  const std::vector<std::string> required = {
    "phase2_enabled", "phase3_enabled", "state", "fault_reason",
    "validation_epoch", "generation", "parent_generation",
    "last_candidate_sequence", "last_candidate_timestamp_ns",
    "last_published_sequence", "last_published_timestamp_ns",
    "correction_age_s", "imu_age_s", "lidar_age_s",
    "imu_buffer_count", "imu_buffer_span_s", "pending_watermark",
    "catch_up_count", "worker_queue_depth", "worker_queue_max_depth",
    "worker_last_processing_s", "worker_max_processing_s", "publish_count",
    "reject_count", "drop_count", "supersede_count", "timeout_count",
    "recovery_healthy_s", "recovery_healthy_required_s",
    "recovery_progress"};
  for (const auto & key : required) {
    expect(!value(message, key).empty(), "missing diagnostic key: " + key);
  }
  expect(message.header.stamp.sec == 123 && message.header.stamp.nanosec == 456,
    "diagnostic header must use its independent supplied ROS time");
}

void test_fault_recovery_state_and_timeout_count()
{
  HighRateEvPropagator::Config config;
  config.enabled = true;
  config.recovery_healthy_s = 0.020;
  HighRateEvPropagator propagator(config);
  HighRateEvDiagnosticInput input;
  input.phase2_enabled = true;
  input.phase3_enabled = true;

  input.propagator = propagator.fixed_diagnostics(1.0, 1.0);
  expect(value(make_message(input), "state") == "INITIALIZING",
    "startup diagnostic state must be INITIALIZING");

  expect(recovery_imu(propagator, 0.995, 1.000), "initial predecessor");
  expect(recovery_imu(propagator, 1.010, 1.015), "initial tail");
  expect(propagator.accept_posterior(recovery_snapshot(1, 1.000), 1.015, 1.015),
    "initial posterior");
  expect(propagator.process(1.015, 1.015), "initial process");
  input.propagator = propagator.fixed_diagnostics(1.015, 1.015);
  expect(value(make_message(input), "state") == "HEALTHY",
    "active diagnostic state must be HEALTHY");

  expect(!propagator.check_timeouts(1.200, 1.200), "timeout must fault");
  input.propagator = propagator.fixed_diagnostics(1.200, 1.200);
  Phase3DiagnosticsStore store;
  store.observe_propagator(input.propagator);
  input.publisher = store.snapshot();
  expect(value(make_message(input), "state") == "FAULT",
    "pre-replay timeout diagnostic state must be FAULT");
  expect(value(make_message(input), "fault_reason") == "IMU_TIMEOUT",
    "fault reason must identify IMU timeout");
  expect(input.publisher.timeout_count == 1, "timeout counter must increment once");
  store.observe_propagator(input.propagator);
  expect(store.snapshot().timeout_count == 1,
    "same timeout validation epoch must not be counted twice");

  expect(recovery_imu(propagator, 1.205, 1.210), "recovery predecessor");
  expect(recovery_imu(propagator, 1.215, 1.220), "recovery tail");
  expect(propagator.accept_posterior(recovery_snapshot(2, 1.210), 1.220, 1.220),
    "recovery posterior");
  expect(propagator.process(1.220, 1.220), "recovery replay");
  input.propagator = propagator.fixed_diagnostics(1.220, 1.220);
  expect(value(make_message(input), "state") == "RECOVERING",
    "replay health wait diagnostic state must be RECOVERING");
  expect(input.propagator.recovery_progress == 0.0,
    "recovery progress must start at zero");

  expect(recovery_imu(propagator, 1.234, 1.245), "post-recovery IMU");
  expect(propagator.process(1.245, 1.245), "post-recovery process");
  input.propagator = propagator.fixed_diagnostics(1.245, 1.245);
  expect(value(make_message(input), "state") == "HEALTHY",
    "completed recovery diagnostic state must return to HEALTHY");
}

void test_queue_publish_drop_and_correspondence()
{
  Phase3DiagnosticsStore store;
  store.record_worker_processing(0.001);
  store.record_candidate(9, 900);
  store.record_prepared(10, 1000);
  store.record_prepared(11, 1100);
  store.record_reject();
  store.record_drop();
  auto snapshot = store.snapshot();
  expect(snapshot.worker_queue_depth == 1 &&
    snapshot.worker_queue_max_depth == 1,
    "single-slot worker queue depth must be one");
  expect(snapshot.supersede_count == 1 && snapshot.drop_count == 2 &&
    snapshot.reject_count == 1,
    "supersede, drop, and reject counters must be exact");
  expect(snapshot.worker_last_processing_s == 0.001 &&
    snapshot.worker_max_processing_s == 0.001,
    "worker processing duration must be retained");

  store.record_prepared(12, 1200);
  store.record_publish(11, 1100);
  snapshot = store.snapshot();
  expect(snapshot.publish_count == 1 && snapshot.worker_queue_depth == 1,
    "publishing an older prepared item must preserve the newer queued item");
  expect(snapshot.last_published_sequence == 11 &&
    snapshot.last_published_timestamp_ns == 1100,
    "published sequence and integer IMU timestamp must remain paired");
  store.record_publish(12, 1200);
  snapshot = store.snapshot();
  expect(snapshot.worker_queue_depth == 0 &&
    snapshot.last_candidate_sequence == snapshot.last_published_sequence &&
    snapshot.last_candidate_timestamp_ns == snapshot.last_published_timestamp_ns,
    "publishing the current candidate must drain its queue entry");
}

void test_concurrent_snapshot_consistency()
{
  Phase3DiagnosticsStore store;
  std::atomic<bool> done{false};
  std::atomic<bool> consistent{true};
  std::thread writer([&]() {
      for (std::uint64_t sequence = 1; sequence <= 20000; ++sequence) {
        const std::int64_t stamp = static_cast<std::int64_t>(sequence * 100);
        store.record_prepared(sequence, stamp);
        store.record_publish(sequence, stamp);
      }
      done.store(true, std::memory_order_release);
    });
  std::thread reader([&]() {
      while (!done.load(std::memory_order_acquire)) {
        const auto snapshot = store.snapshot();
        if (snapshot.last_published_sequence > snapshot.last_candidate_sequence ||
          snapshot.last_published_timestamp_ns > snapshot.last_candidate_timestamp_ns ||
          snapshot.worker_queue_depth > 1 ||
          snapshot.worker_queue_max_depth > 1)
        {
          consistent.store(false, std::memory_order_release);
        }
      }
    });
  writer.join();
  reader.join();
  expect(consistent.load(std::memory_order_acquire),
    "concurrent fixed-size diagnostic snapshots must remain internally consistent");
}

void test_concurrent_propagator_snapshot_consistency()
{
  HighRateEvPropagator::Config config;
  config.enabled = true;
  HighRateEvPropagator propagator(config);
  std::atomic<bool> start{false};
  std::atomic<bool> done{false};
  std::atomic<bool> consistent{true};
  std::thread writer([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      for (std::uint64_t index = 1; index <= 300; ++index) {
        const double timestamp = 2.0 + 0.001 * static_cast<double>(index);
        if (!propagator.ingest_imu(
            timestamp, Eigen::Vector3d(0.0, 0.0, 9.81),
            Eigen::Vector3d::Zero(), timestamp, timestamp))
        {
          consistent.store(false, std::memory_order_release);
          break;
        }
      }
      done.store(true, std::memory_order_release);
    });
  std::thread reader([&]() {
      start.store(true, std::memory_order_release);
      while (!done.load(std::memory_order_acquire)) {
        const auto snapshot = propagator.fixed_diagnostics(2.5, 2.5);
        if (snapshot.imu_buffer_count > config.imu_buffer_max_count ||
          snapshot.imu_buffer_span_s < 0.0 ||
          (snapshot.imu_buffer_count < 2 &&
          snapshot.imu_buffer_span_s != 0.0) ||
          (snapshot.imu_buffer_count > 1 &&
          snapshot.imu_buffer_span_s <= 0.0))
        {
          consistent.store(false, std::memory_order_release);
        }
      }
    });
  writer.join();
  reader.join();
  expect(consistent.load(std::memory_order_acquire),
    "propagator scalar snapshot must remain coherent during concurrent IMU ingest");
}

std::shared_ptr<const Candidate> candidate(std::uint64_t sequence, double timestamp)
{
  std::shared_ptr<Candidate> result(new Candidate());
  result->generation = 1;
  result->imu_sequence = sequence;
  result->timestamp = timestamp;
  result->covariance.setIdentity();
  return std::shared_ptr<const Candidate>(result);
}

void test_diagnostic_build_does_not_change_odometry_gate()
{
  HighRateEvOdometryGate::Config config;
  config.enabled = true;
  HighRateEvOdometryGate reference(config);
  HighRateEvOdometryGate interleaved(config);
  fast_lio::PropagatedOdometry reference_output;
  fast_lio::PropagatedOdometry interleaved_output;
  HighRateEvDiagnosticInput diagnostic_input;
  diagnostic_input.phase2_enabled = true;
  diagnostic_input.phase3_enabled = true;
  diagnostic_input.propagator.health = HighRateEvPropagator::Health::kHealthy;

  for (std::uint64_t index = 0; index < 8; ++index) {
    const auto current = candidate(index + 1, 5.0 + 0.025 * index);
    const bool reference_result = reference.try_build(current, &reference_output);
    (void)make_message(diagnostic_input);
    const bool interleaved_result = interleaved.try_build(current, &interleaved_output);
    expect(reference_result == interleaved_result,
      "diagnostic construction must not change Odometry publish eligibility");
  }
}

}  // namespace

int main()
{
  test_default_off_and_required_keys();
  test_fault_recovery_state_and_timeout_count();
  test_queue_publish_drop_and_correspondence();
  test_concurrent_snapshot_consistency();
  test_concurrent_propagator_snapshot_consistency();
  test_diagnostic_build_does_not_change_odometry_gate();
  if (failures != 0) {
    std::cerr << failures << " Phase 3.5 assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "Phase 3.5 targeted diagnostic tests passed\n";
  return EXIT_SUCCESS;
}

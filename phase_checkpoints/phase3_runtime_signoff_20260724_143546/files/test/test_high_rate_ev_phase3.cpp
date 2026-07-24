#include "high_rate_ev_odometry.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

namespace
{
using fast_lio::HighRateEvOdometryGate;
using fast_lio::HighRateEvPropagator;
using fast_lio::PropagatedOdometry;
using fast_lio::ExactImuStampIndex;
using Candidate = HighRateEvPropagator::Candidate;
using Covariance = HighRateEvPropagator::Covariance;

int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void expect_near(double actual, double expected, double tolerance, const std::string & message)
{
  expect(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

HighRateEvOdometryGate::Config enabled_gate_config()
{
  HighRateEvOdometryGate::Config config;
  config.enabled = true;
  return config;
}

std::shared_ptr<const Candidate> make_candidate(
  std::uint64_t sequence, double timestamp,
  const Eigen::Matrix3d & rotation_world_body = Eigen::Matrix3d::Identity(),
  const Eigen::Vector3d & velocity_world = Eigen::Vector3d::Zero(),
  const Covariance & covariance = Covariance::Identity())
{
  std::shared_ptr<Candidate> candidate(new Candidate());
  candidate->generation = 7;
  candidate->parent_generation = 6;
  candidate->validation_epoch = 11;
  candidate->imu_sequence = sequence;
  candidate->timestamp = timestamp;
  candidate->state.pos << 1.25, -2.5, 0.75;
  candidate->state.rot = SO3(rotation_world_body);
  candidate->state.vel = velocity_world;
  candidate->covariance = covariance;
  return std::shared_ptr<const Candidate>(candidate);
}

Eigen::Matrix<double, 6, 6> pose_covariance_from_output(
  const PropagatedOdometry & output)
{
  Eigen::Matrix<double, 6, 6> result;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      result(row, column) =
        output.pose_covariance[static_cast<std::size_t>(row * 6 + column)];
    }
  }
  return result;
}

Eigen::Matrix3d velocity_covariance_from_output(const PropagatedOdometry & output)
{
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result(row, column) =
        output.twist_covariance[static_cast<std::size_t>(row * 6 + column)];
    }
  }
  return result;
}

Covariance nontrivial_covariance()
{
  Covariance seed;
  for (int row = 0; row < seed.rows(); ++row) {
    for (int column = 0; column < seed.cols(); ++column) {
      seed(row, column) =
        2.0e-3 * std::sin(
        0.19 * static_cast<double>((row + 1) * (column + 3)));
    }
  }
  Covariance covariance = seed * seed.transpose();
  covariance.diagonal().array() += 3.0e-4;
  return covariance;
}

void test_default_off_and_fail_closed_config()
{
  HighRateEvOdometryGate::Config disabled_config;
  expect(!fast_lio::make_high_rate_ev_odometry_gate_if_enabled(disabled_config),
    "default-off factory must construct no Phase 3 gate");

  HighRateEvOdometryGate disabled;
  PropagatedOdometry output;
  expect(!disabled.enabled(), "default Phase 3 gate must be disabled");
  expect(!disabled.try_build(make_candidate(1, 1.0), &output),
    "disabled Phase 3 gate must not map or publish a candidate");

  auto invalid = enabled_gate_config();
  invalid.publish_interval_s = 0.019;
  expect(!fast_lio::make_high_rate_ev_odometry_gate_if_enabled(invalid),
    "publish intervals above 50 Hz must fail closed");
  invalid.publish_interval_s = 0.034;
  expect(!fast_lio::make_high_rate_ev_odometry_gate_if_enabled(invalid),
    "publish intervals below 30 Hz must fail closed");
}

void test_exact_imu_stamp_index_preserves_integer_nanoseconds()
{
  ExactImuStampIndex index(2);
  expect(index.valid(), "positive exact-stamp capacity must be valid");
  expect(index.record(1700000000.1234567, 1700000000123456789LL),
    "first exact corrected IMU stamp must be accepted");
  expect(index.record(1700000000.1334567, 1700000000133456789LL),
    "strictly newer exact corrected IMU stamp must be accepted");

  std::int64_t nanoseconds = 0;
  expect(index.lookup(1700000000.1234567, &nanoseconds) &&
    nanoseconds == 1700000000123456789LL,
    "lookup must return the recorded integer nanoseconds without reconstruction");
  expect(!index.record(1700000000.1334567, 1700000000133456790LL),
    "duplicate source timestamp must be rejected");
  expect(index.record(1700000000.1434567, 1700000000143456789LL),
    "third exact corrected IMU stamp must be accepted");
  expect(!index.lookup(1700000000.1234567, &nanoseconds),
    "bounded exact-stamp index must retire its oldest entry");
  expect(!index.lookup(1700000000.1434567, nullptr),
    "lookup must reject a null output");
}

void test_frames_body_velocity_exact_stamp_and_rate_gate()
{
  HighRateEvOdometryGate gate(enabled_gate_config());
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto first = make_candidate(
    40, 100.123456789, rotation, Eigen::Vector3d(0.0, 2.0, -0.5));
  PropagatedOdometry output;
  expect(gate.try_build(first, &output), "first fresh HEALTHY candidate must publish");
  expect(output.frame_id == "camera_init" && output.child_frame_id == "body",
    "message frames must be camera_init to body");
  expect(output.timestamp == 100.123456789,
    "published timestamp must exactly equal the integrated IMU timestamp");
  expect((output.position_world - Eigen::Vector3d(1.25, -2.5, 0.75)).norm() == 0.0,
    "world position must be copied without a frame substitution");
  expect((output.orientation_world_body.toRotationMatrix() - rotation).norm() < 1.0e-12,
    "orientation must remain R_world_body");
  expect(output.orientation_world_body.coeffs().allFinite() &&
    std::abs(output.orientation_world_body.norm() - 1.0) < 1.0e-12,
    "published quaternion must be finite and normalized");
  expect((output.linear_velocity_body - Eigen::Vector3d(2.0, 0.0, -0.5)).norm() < 1.0e-12,
    "linear velocity must be rotated into body FLU");

  expect(!gate.try_build(first, &output), "same immutable candidate must not repeat");
  expect(!gate.try_build(make_candidate(41, 100.133456789), &output),
    "fresh IMU pairs faster than 50 Hz must not publish");
  expect(gate.try_build(make_candidate(42, 100.148456789), &output),
    "a fresh pair at the configured 40 Hz interval must publish");
  expect(!gate.try_build(make_candidate(41, 100.200000000), &output),
    "an older sequence must remain suppressed");
  expect(!gate.try_build(make_candidate(43, 100.140000000), &output),
    "a backward source timestamp must remain suppressed");
}

void test_pose_covariance_world_rotation_and_cross_terms()
{
  const Covariance covariance = nontrivial_covariance();
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(0.37, Eigen::Vector3d(0.3, -0.4, 0.8).normalized())).
    toRotationMatrix();
  HighRateEvOdometryGate gate(enabled_gate_config());
  PropagatedOdometry output;
  expect(gate.try_build(make_candidate(1, 2.0, rotation, Eigen::Vector3d::Zero(),
    covariance), &output), "valid nontrivial covariance must publish");

  Eigen::Matrix<double, 6, 6> expected = Eigen::Matrix<double, 6, 6>::Zero();
  expected.block<3, 3>(0, 0) = covariance.block<3, 3>(0, 0);
  expected.block<3, 3>(0, 3) =
    covariance.block<3, 3>(0, 3) * rotation.transpose();
  expected.block<3, 3>(3, 0) =
    rotation * covariance.block<3, 3>(3, 0);
  expected.block<3, 3>(3, 3) =
    rotation * covariance.block<3, 3>(3, 3) * rotation.transpose();
  expect((pose_covariance_from_output(output) - expected).norm() < 1.0e-11,
    "pose covariance must use ROS position/rotation order and preserve cross terms");
}

Eigen::Matrix<double, 3, 23> finite_difference_velocity_jacobian(
  const Eigen::Matrix3d & rotation_world_body,
  const Eigen::Vector3d & velocity_world)
{
  constexpr double epsilon = 1.0e-7;
  Eigen::Matrix<double, 3, 23> jacobian =
    Eigen::Matrix<double, 3, 23>::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d direction = Eigen::Vector3d::Unit(axis);
    const Eigen::Matrix3d plus_rotation =
      rotation_world_body *
      Eigen::AngleAxisd(epsilon, direction).toRotationMatrix();
    const Eigen::Matrix3d minus_rotation =
      rotation_world_body *
      Eigen::AngleAxisd(-epsilon, direction).toRotationMatrix();
    jacobian.col(3 + axis) =
      (plus_rotation.transpose() * velocity_world -
      minus_rotation.transpose() * velocity_world) / (2.0 * epsilon);
    jacobian.col(12 + axis) =
      (rotation_world_body.transpose() *
      (velocity_world + epsilon * direction) -
      rotation_world_body.transpose() *
      (velocity_world - epsilon * direction)) / (2.0 * epsilon);
  }
  return jacobian;
}

void test_velocity_covariance_uses_full_right_error_jacobian()
{
  const Covariance covariance = nontrivial_covariance();
  const Eigen::Matrix3d rotation =
    (Eigen::AngleAxisd(-0.61, Eigen::Vector3d(0.7, 0.2, -0.1).normalized())).
    toRotationMatrix();
  const Eigen::Vector3d velocity_world(2.1, -0.4, 0.8);
  HighRateEvOdometryGate gate(enabled_gate_config());
  PropagatedOdometry output;
  expect(gate.try_build(make_candidate(5, 4.0, rotation, velocity_world, covariance),
    &output), "velocity covariance fixture must publish");

  const Eigen::Matrix<double, 3, 23> numerical_jacobian =
    finite_difference_velocity_jacobian(rotation, velocity_world);
  const Eigen::Matrix3d expected =
    numerical_jacobian * covariance * numerical_jacobian.transpose();
  expect((velocity_covariance_from_output(output) - expected).norm() < 2.0e-9,
    "body velocity covariance must include attitude, velocity, and cross terms");
}

void test_unknown_angular_rate_and_covariance_rejection()
{
  HighRateEvOdometryGate gate(enabled_gate_config());
  PropagatedOdometry output;
  expect(gate.try_build(make_candidate(1, 6.0), &output),
    "valid candidate must publish for angular-rate contract test");
  expect(output.angular_velocity_body.array().isNaN().all(),
    "unknown angular velocity must remain NaN on all axes");
  for (double value : output.twist_covariance) {
    expect(std::isfinite(value), "twist covariance must remain finite");
  }
  expect(output.twist_covariance[21] == 1.0e6 &&
    output.twist_covariance[28] == 1.0e6 &&
    output.twist_covariance[35] == 1.0e6,
    "unknown angular axes must retain conservative finite variance");
  for (int linear = 0; linear < 3; ++linear) {
    for (int angular = 3; angular < 6; ++angular) {
      expect(output.twist_covariance[linear * 6 + angular] == 0.0 &&
        output.twist_covariance[angular * 6 + linear] == 0.0,
        "linear/angular covariance cross blocks must remain zero");
    }
  }

  Covariance asymmetric = Covariance::Identity();
  asymmetric(0, 1) = 1.0;
  expect(!gate.try_build(make_candidate(2, 6.030, Eigen::Matrix3d::Identity(),
    Eigen::Vector3d::Zero(), asymmetric), &output),
    "asymmetric candidate covariance must not publish");
  Covariance indefinite = Covariance::Identity();
  indefinite(12, 12) = -1.0;
  expect(!gate.try_build(make_candidate(3, 6.060, Eigen::Matrix3d::Identity(),
    Eigen::Vector3d::Zero(), indefinite), &output),
    "indefinite candidate covariance must not publish");
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

void test_fault_suppression_and_recovery_publish()
{
  HighRateEvPropagator::Config propagator_config;
  propagator_config.enabled = true;
  propagator_config.recovery_healthy_s = 0.020;
  HighRateEvPropagator propagator(propagator_config);
  HighRateEvOdometryGate gate(enabled_gate_config());
  PropagatedOdometry output;

  expect(recovery_imu(propagator, 0.995, 1.000), "initial predecessor IMU");
  expect(recovery_imu(propagator, 1.010, 1.015), "initial propagated IMU");
  expect(propagator.accept_posterior(recovery_snapshot(1, 1.000), 1.015, 1.015),
    "initial posterior");
  expect(propagator.process(1.015, 1.015), "initial candidate processing");
  const auto before_fault = propagator.candidate();
  expect(before_fault && gate.try_build(before_fault, &output),
    "HEALTHY Phase 2 candidate must publish before timeout");

  expect(!propagator.check_timeouts(1.200, 1.200),
    "IMU timeout must fail the watchdog");
  expect(!propagator.candidate(),
    "FAULT must clear the immutable candidate immediately");
  expect(!gate.try_build(propagator.candidate(), &output),
    "FAULT must produce no stale Phase 3 output");

  expect(recovery_imu(propagator, 1.205, 1.210), "recovery predecessor IMU");
  expect(recovery_imu(propagator, 1.215, 1.220), "recovery tail IMU");
  expect(propagator.accept_posterior(recovery_snapshot(2, 1.210), 1.220, 1.220),
    "recovery posterior");
  expect(propagator.process(1.220, 1.220), "recovery replay");
  expect(!propagator.candidate() &&
    propagator.diagnostics().recovery_waiting,
    "recovery_healthy_s wait must suppress publication");
  expect(!gate.try_build(propagator.candidate(), &output),
    "recovery wait must retain no old output");

  expect(recovery_imu(propagator, 1.234, 1.245), "post-recovery fresh IMU");
  expect(propagator.process(1.245, 1.245), "post-recovery processing");
  const auto recovered = propagator.candidate();
  expect(recovered && recovered->imu_sequence > before_fault->imu_sequence &&
    recovered->timestamp > before_fault->timestamp,
    "recovery candidate must advance the preserved global pair");
  expect(gate.try_build(recovered, &output),
    "new HEALTHY candidate must resume publication after recovery_healthy_s");
  expect(!gate.try_build(before_fault, &output),
    "pre-FAULT candidate must never publish after recovery");
}

}  // namespace

int main()
{
  test_default_off_and_fail_closed_config();
  test_exact_imu_stamp_index_preserves_integer_nanoseconds();
  test_frames_body_velocity_exact_stamp_and_rate_gate();
  test_pose_covariance_world_rotation_and_cross_terms();
  test_velocity_covariance_uses_full_right_error_jacobian();
  test_unknown_angular_rate_and_covariance_rejection();
  test_fault_suppression_and_recovery_publish();
  if (failures != 0) {
    std::cerr << failures << " Phase 3 assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "Phase 3 targeted publisher tests passed\n";
  return EXIT_SUCCESS;
}

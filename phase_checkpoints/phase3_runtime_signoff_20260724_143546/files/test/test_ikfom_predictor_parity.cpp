#include "ikfom_predictor_fixture.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{

using fast_lio_test::Covariance;
using fast_lio_test::IndependentIkfomPredictorFixture;
using fast_lio_test::NoiseDiagonals;
using fast_lio_test::PosteriorSnapshot;
using fast_lio_test::PredictionResult;
using fast_lio_test::RawImuSample;

int failures = 0;

void expect_true(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void expect_near(double actual, double expected, double tolerance, const std::string & message)
{
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
    std::fabs(actual - expected) > tolerance)
  {
    std::cerr << "FAIL: " << message << " actual=" << actual <<
      " expected=" << expected << '\n';
    ++failures;
  }
}

template<typename Callable>
void expect_invalid_argument(Callable callable, const std::string & message)
{
  try {
    callable();
    expect_true(false, message + " must throw std::invalid_argument");
  } catch (const std::invalid_argument &) {
    // Expected fail-closed outcome.
  } catch (...) {
    expect_true(false, message + " threw the wrong exception type");
  }
}

PosteriorSnapshot make_snapshot(double correction_timestamp)
{
  PosteriorSnapshot snapshot;
  snapshot.correction_timestamp = correction_timestamp;
  snapshot.mean_acceleration_norm = 9.81;

  snapshot.state.pos << 1.2, -0.7, 0.4;
  snapshot.state.rot = SO3(
    (Eigen::AngleAxisd(0.21, Eigen::Vector3d(0.2, -0.4, 0.7).normalized())).toRotationMatrix());
  snapshot.state.offset_R_L_I = SO3(
    (Eigen::AngleAxisd(-0.08, Eigen::Vector3d(0.5, 0.1, -0.3).normalized())).toRotationMatrix());
  snapshot.state.offset_T_L_I << 0.04, -0.02, 0.03;
  snapshot.state.vel.setZero();
  snapshot.state.bg.setZero();
  // S2 length is 98090/10000 = 9.809 while G_m_s2 is 9.81. This exact bias
  // makes the deterministic nominal acceleration zero without tolerance tricks.
  snapshot.state.ba << 0.0, 0.0, 0.001;
  const Eigen::Vector3d stationary_acceleration_body(0.0, 0.0, 9.81);
  const Eigen::Vector3d gravity_world =
    -snapshot.state.rot.toRotationMatrix() * stationary_acceleration_body;
  snapshot.state.grav = S2(gravity_world);

  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> seed;
  for (int row = 0; row < state_ikfom::DOF; ++row) {
    for (int column = 0; column < state_ikfom::DOF; ++column) {
      seed(row, column) =
        1.0e-3 * std::sin(0.31 * static_cast<double>((row + 1) * (column + 2)));
    }
  }
  snapshot.covariance = seed * seed.transpose();
  snapshot.covariance.diagonal().array() += 1.0e-4;

  snapshot.noise.gyro << 0.11, 0.12, 0.13;
  snapshot.noise.acceleration << 0.21, 0.22, 0.23;
  snapshot.noise.gyro_bias << 1.1e-4, 1.2e-4, 1.3e-4;
  snapshot.noise.acceleration_bias << 2.1e-4, 2.2e-4, 2.3e-4;
  return snapshot;
}

RawImuSample make_imu(
  std::uint64_t sequence,
  double timestamp,
  const Eigen::Vector3d & acceleration,
  const Eigen::Vector3d & angular_velocity)
{
  RawImuSample sample;
  sample.sequence = sequence;
  sample.timestamp = timestamp;
  sample.acceleration = acceleration;
  sample.angular_velocity = angular_velocity;
  return sample;
}

struct AnalyticGolden
{
  state_ikfom state{};
  Covariance covariance{Covariance::Identity()};
  Eigen::Matrix<double, 12, 12> process_noise{
    Eigen::Matrix<double, 12, 12>::Zero()};
  input_ikfom normalized_input{};
  double dt{};
};

AnalyticGolden make_independent_analytic_golden(
  const PosteriorSnapshot & snapshot,
  const RawImuSample & head,
  const RawImuSample & tail)
{
  AnalyticGolden golden;
  golden.state = snapshot.state;

  // Oracle-side construction is intentionally independent from every helper in
  // ikfom_predictor_fixture.hpp and does not instantiate Filter or call predict().
  const Eigen::Vector3d acceleration_midpoint =
    (head.acceleration + tail.acceleration) * 0.5;
  const Eigen::Vector3d gyro_midpoint =
    (head.angular_velocity + tail.angular_velocity) * 0.5;
  golden.normalized_input.acc =
    acceleration_midpoint * 9.81 / snapshot.mean_acceleration_norm;
  golden.normalized_input.gyro = gyro_midpoint;
  golden.dt = tail.timestamp - snapshot.correction_timestamp;

  golden.process_noise.setZero();
  golden.process_noise.block<3, 3>(0, 0).diagonal() = snapshot.noise.gyro;
  golden.process_noise.block<3, 3>(3, 3).diagonal() = snapshot.noise.acceleration;
  golden.process_noise.block<3, 3>(6, 6).diagonal() = snapshot.noise.gyro_bias;
  golden.process_noise.block<3, 3>(9, 9).diagonal() =
    snapshot.noise.acceleration_bias;

  const vect3 unbiased_angular_velocity =
    golden.normalized_input.gyro - snapshot.state.bg;
  const vect3 unbiased_acceleration =
    golden.normalized_input.acc - snapshot.state.ba;
  const Eigen::Vector3d world_acceleration =
    snapshot.state.rot.toRotationMatrix() *
    unbiased_acceleration + snapshot.state.grav.vec;

  // Independent nominal-state oracle for IKFoM x.oplus(f, dt).
  golden.state.pos = snapshot.state.pos + snapshot.state.vel * golden.dt;
  golden.state.rot = snapshot.state.rot;
  golden.state.rot.boxplus(
    MTK::vectview<const double, 3>(unbiased_angular_velocity), golden.dt);
  golden.state.vel = snapshot.state.vel + world_acceleration * golden.dt;

  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> raw_f =
    Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Zero();
  raw_f.block<3, 3>(0, 12).setIdentity();
  const Eigen::Matrix3d rotation = snapshot.state.rot.toRotationMatrix();
  raw_f.block<3, 3>(12, 3) = -rotation * MTK::hat(unbiased_acceleration);
  raw_f.block<3, 3>(12, 18) = -rotation;
  Eigen::Matrix<double, 3, 2> gravity_jacobian;
  Eigen::Matrix<double, 2, 1> zero_gravity_error =
    Eigen::Matrix<double, 2, 1>::Zero();
  state_ikfom state_for_gravity = snapshot.state;
  state_for_gravity.S2_Mx(gravity_jacobian, zero_gravity_error, 21);
  raw_f.block<3, 2>(12, 21) = gravity_jacobian;
  raw_f.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, state_ikfom::DOF, 12> raw_noise_map =
    Eigen::Matrix<double, state_ikfom::DOF, 12>::Zero();
  raw_noise_map.block<3, 3>(12, 3) = -rotation;
  raw_noise_map.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  raw_noise_map.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  raw_noise_map.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();

  // Reproduce the SO3 error-coordinate retraction algebra independently from
  // Filter::predict. Other SO3/S2 nominal derivatives are zero in this model.
  const vect3 negative_rotation_increment =
    -unbiased_angular_velocity * golden.dt;
  SO3 attitude_retraction;
  attitude_retraction.w() = MTK::exp<double, 3>(
    attitude_retraction.vec(), negative_rotation_increment, 0.5);
  const Eigen::Matrix3d attitude_error_map =
    attitude_retraction.toRotationMatrix();
  const Eigen::Matrix3d attitude_derivative_map =
    MTK::A_matrix(negative_rotation_increment);

  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> mapped_f = raw_f;
  Eigen::Matrix<double, state_ikfom::DOF, 12> mapped_noise_map = raw_noise_map;
  mapped_f.block<3, state_ikfom::DOF>(3, 0) =
    attitude_derivative_map * raw_f.block<3, state_ikfom::DOF>(3, 0);
  mapped_noise_map.block<3, 12>(3, 0) =
    attitude_derivative_map * raw_noise_map.block<3, 12>(3, 0);

  Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> transition =
    Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF>::Identity();
  transition.block<3, 3>(3, 3) = attitude_error_map;
  Eigen::Matrix<double, 2, 3> gravity_nx;
  Eigen::Matrix<double, 3, 2> gravity_mx;
  golden.state.S2_Nx_yy(gravity_nx, 21);
  state_for_gravity.S2_Mx(gravity_mx, zero_gravity_error, 21);
  transition.block<2, 2>(21, 21) = gravity_nx * gravity_mx;
  transition += mapped_f * golden.dt;
  const Eigen::Matrix<double, state_ikfom::DOF, 12> discrete_noise_map =
    mapped_noise_map * golden.dt;
  golden.covariance =
    transition * snapshot.covariance * transition.transpose() +
    discrete_noise_map * golden.process_noise * discrete_noise_map.transpose();
  return golden;
}

void expect_state_elementwise_equal(
  const state_ikfom & actual,
  const state_ikfom & expected,
  const std::string & label)
{
  Eigen::Matrix<double, state_ikfom::DOF, 1> difference;
  actual.boxminus(
    MTK::vectview<double, state_ikfom::DOF>(difference), expected);
  for (int index = 0; index < state_ikfom::DOF; ++index) {
    expect_near(difference(index), 0.0, 1.0e-13,
      label + " state error index " + std::to_string(index));
  }
}

template<int Rows, int Columns>
void expect_matrix_elementwise_equal(
  const Eigen::Matrix<double, Rows, Columns> & actual,
  const Eigen::Matrix<double, Rows, Columns> & expected,
  double tolerance,
  const std::string & label)
{
  for (int row = 0; row < Rows; ++row) {
    for (int column = 0; column < Columns; ++column) {
      expect_near(actual(row, column), expected(row, column), tolerance,
        label + " (" + std::to_string(row) + "," + std::to_string(column) + ")");
    }
  }
}

void expect_input_equal(
  const input_ikfom & actual,
  const input_ikfom & expected,
  const std::string & label)
{
  for (int axis = 0; axis < 3; ++axis) {
    expect_near(actual.acc(axis), expected.acc(axis), 0.0,
      label + " acceleration axis " + std::to_string(axis));
    expect_near(actual.gyro(axis), expected.gyro(axis), 0.0,
      label + " gyro axis " + std::to_string(axis));
  }
}

void check_case(
  const std::string & label,
  const PosteriorSnapshot & snapshot,
  const RawImuSample & head,
  const RawImuSample & tail,
  double covariance_tolerance = 2.0e-15)
{
  IndependentIkfomPredictorFixture fixture(snapshot);
  const PredictionResult independent =
    fixture.predict_first_partial_interval(&head, &tail);
  const AnalyticGolden golden =
    make_independent_analytic_golden(snapshot, head, tail);

  expect_state_elementwise_equal(independent.state, golden.state, label);
  expect_matrix_elementwise_equal<state_ikfom::DOF, state_ikfom::DOF>(
    independent.covariance, golden.covariance, covariance_tolerance,
    label + " covariance");
  expect_matrix_elementwise_equal<12, 12>(
    independent.process_noise, golden.process_noise, 0.0, label + " Q");
  expect_input_equal(independent.normalized_input, golden.normalized_input, label);
  expect_near(independent.dt, golden.dt, 0.0, label + " dt");
  expect_true(
    independent.integrated_sequence == tail.sequence,
    label + " must integrate exactly the tail sequence");
}

void test_snapshot_copy_is_complete_and_independent()
{
  PosteriorSnapshot source = make_snapshot(100.0);
  const PosteriorSnapshot baseline = source;
  const PosteriorSnapshot copy = source;
  source.correction_timestamp += 10.0;
  source.mean_acceleration_norm += 1.0;
  source.state.pos.array() += 1.0;
  source.state.rot = SO3(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
  source.state.offset_R_L_I = SO3(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()));
  source.state.offset_T_L_I.array() -= 1.0;
  source.state.vel.array() += 2.0;
  source.state.bg.array() += 3.0;
  source.state.ba.array() += 4.0;
  source.state.grav = S2(1.0, 2.0, -8.0);
  source.covariance.setConstant(7.0);
  source.noise.gyro.setConstant(8.0);
  source.noise.acceleration.setConstant(9.0);
  source.noise.gyro_bias.setConstant(10.0);
  source.noise.acceleration_bias.setConstant(11.0);

  expect_near(copy.correction_timestamp, baseline.correction_timestamp, 0.0,
    "snapshot correction timestamp must be by value");
  expect_near(copy.mean_acceleration_norm, baseline.mean_acceleration_norm, 0.0,
    "snapshot acceleration norm must be by value");
  expect_state_elementwise_equal(copy.state, baseline.state, "snapshot complete state copy");
  expect_matrix_elementwise_equal<state_ikfom::DOF, state_ikfom::DOF>(
    copy.covariance, baseline.covariance, 0.0, "snapshot complete P copy");
  for (int axis = 0; axis < 3; ++axis) {
    expect_near(copy.noise.gyro(axis), baseline.noise.gyro(axis), 0.0,
      "snapshot gyro Q copy axis " + std::to_string(axis));
    expect_near(copy.noise.acceleration(axis), baseline.noise.acceleration(axis), 0.0,
      "snapshot acceleration Q copy axis " + std::to_string(axis));
    expect_near(copy.noise.gyro_bias(axis), baseline.noise.gyro_bias(axis), 0.0,
      "snapshot gyro bias Q copy axis " + std::to_string(axis));
    expect_near(
      copy.noise.acceleration_bias(axis), baseline.noise.acceleration_bias(axis), 0.0,
      "snapshot acceleration bias Q copy axis " + std::to_string(axis));
  }
}

void test_equal_timestamp_boundary()
{
  const PosteriorSnapshot snapshot = make_snapshot(100.0);
  const RawImuSample head = make_imu(
    40, 100.0, {0.2, -0.1, 9.70}, {0.01, -0.02, 0.03});
  const RawImuSample tail = make_imu(
    41, 100.006, {-0.2, 0.1, 9.92}, {-0.01, 0.02, -0.03});
  check_case("equal timestamp", snapshot, head, tail);
}

void test_straddling_partial_interval()
{
  const PosteriorSnapshot snapshot = make_snapshot(200.0);
  const RawImuSample head = make_imu(
    80, 199.996, {-0.3, 0.2, 9.62}, {-0.02, 0.04, 0.01});
  const RawImuSample tail = make_imu(
    81, 200.005, {0.3, -0.2, 10.0}, {0.02, -0.04, -0.01});
  check_case("straddling interval", snapshot, head, tail);

  IndependentIkfomPredictorFixture fixture(snapshot);
  const PredictionResult result = fixture.predict_first_partial_interval(&head, &tail);
  expect_near(result.dt, 0.005, 1.0e-14,
    "straddling interval must integrate tail minus correction, not tail minus head");
}

void test_correction_newer_than_latest_imu_then_new_tail()
{
  const PosteriorSnapshot snapshot = make_snapshot(300.0);
  const RawImuSample latest_before = make_imu(
    120, 299.992, {0.0, -0.2, 9.68}, {0.03, 0.01, -0.02});
  const RawImuSample new_tail = make_imu(
    121, 300.004, {0.0, 0.2, 9.94}, {-0.03, -0.01, 0.02});
  check_case("correction newer than latest IMU", snapshot, latest_before, new_tail);
}

void test_nonzero_world_acceleration_golden()
{
  const PosteriorSnapshot snapshot = make_snapshot(350.0);
  const RawImuSample head = make_imu(
    140, 349.997, {0.6, -0.3, 9.85}, {0.01, -0.02, 0.03});
  const RawImuSample tail = make_imu(
    141, 350.006, {0.2, -0.1, 9.97}, {-0.01, 0.02, -0.03});
  check_case("nonzero world acceleration", snapshot, head, tail);

  const AnalyticGolden golden = make_independent_analytic_golden(snapshot, head, tail);
  const Eigen::Vector3d world_acceleration =
    snapshot.state.rot.toRotationMatrix() *
    (golden.normalized_input.acc - snapshot.state.ba) + snapshot.state.grav.vec;
  expect_true(world_acceleration.norm() > 0.1,
    "nonzero acceleration golden must exercise translational dynamics");
}

void test_nonzero_angular_velocity_golden()
{
  const PosteriorSnapshot snapshot = make_snapshot(375.0);
  const RawImuSample head = make_imu(
    150, 374.997, {0.2, -0.1, 9.70}, {0.25, -0.15, 0.08});
  const RawImuSample tail = make_imu(
    151, 375.006, {-0.2, 0.1, 9.92}, {0.35, -0.25, 0.12});
  // The independent closed-form SO3 Jacobian and IKFoM's quaternion path use
  // different floating-point evaluation orders; retain a bounded elementwise
  // tolerance for this deliberately nonzero rotational increment.
  check_case("nonzero angular velocity", snapshot, head, tail, 2.0e-8);

  const AnalyticGolden golden = make_independent_analytic_golden(snapshot, head, tail);
  expect_true(golden.normalized_input.gyro.norm() > 0.1,
    "nonzero angular golden must exercise SO3 state and covariance retraction");
}

void test_tail_equal_to_correction_is_context_only()
{
  const PosteriorSnapshot snapshot = make_snapshot(400.0);
  const RawImuSample head = make_imu(
    160, 399.995, {0.1, 0.0, 9.75}, {0.01, 0.0, -0.01});
  const RawImuSample tail_at_correction = make_imu(
    161, 400.0, {-0.1, 0.0, 9.87}, {-0.01, 0.0, 0.01});
  IndependentIkfomPredictorFixture fixture(snapshot);
  expect_invalid_argument(
    [&fixture, &head, &tail_at_correction]() {
      fixture.predict_first_partial_interval(&head, &tail_at_correction);
    },
    "tail equal to correction");

  const RawImuSample new_tail = make_imu(
    162, 400.006, {0.1, 0.0, 9.75}, {0.01, 0.0, -0.01});
  const PredictionResult result =
    fixture.predict_first_partial_interval(&tail_at_correction, &new_tail);
  const AnalyticGolden golden =
    make_independent_analytic_golden(snapshot, tail_at_correction, new_tail);
  expect_state_elementwise_equal(
    result.state, golden.state, "tail at correction becomes next head");
  expect_matrix_elementwise_equal<state_ikfom::DOF, state_ikfom::DOF>(
    result.covariance, golden.covariance, 2.0e-15,
    "tail at correction continuity covariance");
  expect_near(
    result.dt, new_tail.timestamp - snapshot.correction_timestamp, 0.0,
    "new tail must integrate exactly once from correction");
  expect_true(result.integrated_sequence == new_tail.sequence,
    "only the strictly newer tail sequence may be integrated");
}

void test_no_new_tail_is_not_predictable()
{
  const PosteriorSnapshot snapshot = make_snapshot(500.0);
  const RawImuSample head = make_imu(
    200, 499.995, {0.0, 0.0, 9.81}, {0.0, 0.0, 0.0});
  IndependentIkfomPredictorFixture fixture(snapshot);
  expect_invalid_argument(
    [&fixture, &head]() {
      fixture.predict_first_partial_interval(&head, nullptr);
    },
    "missing new tail");
}

void test_missing_predecessor_is_not_predictable()
{
  const PosteriorSnapshot snapshot = make_snapshot(600.0);
  const RawImuSample tail = make_imu(
    241, 600.005, {0.0, 0.0, 9.81}, {0.0, 0.0, 0.0});
  IndependentIkfomPredictorFixture fixture(snapshot);
  expect_invalid_argument(
    [&fixture, &tail]() {
      fixture.predict_first_partial_interval(nullptr, &tail);
    },
    "missing predecessor");
}

}  // namespace

int main()
{
  test_snapshot_copy_is_complete_and_independent();
  test_equal_timestamp_boundary();
  test_straddling_partial_interval();
  test_correction_newer_than_latest_imu_then_new_tail();
  test_nonzero_world_acceleration_golden();
  test_nonzero_angular_velocity_golden();
  test_tail_equal_to_correction_is_context_only();
  test_no_new_tail_is_not_predictable();
  test_missing_predecessor_is_not_predictable();

  if (failures != 0) {
    std::cerr << failures << " Phase 1 parity assertion(s) failed\n";
    return EXIT_FAILURE;
  }

  std::cout << "Phase 1 IKFoM predictor parity tests passed\n";
  return EXIT_SUCCESS;
}

// SPDX-License-Identifier: GPL-2.0-only

#include <gtest/gtest.h>

#include <limits>

#include <fr_lio/high_rate_odom.hpp>

namespace
{

fr_lio::HighRateOdomState stationary_anchor(double timestamp = 0.0)
{
  fr_lio::HighRateOdomState state;
  state.timestamp = timestamp;
  state.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  state.covariance.setIdentity();
  state.covariance *= 1e-3;
  return state;
}

fr_lio::HighRateImuSample stationary_imu(double timestamp)
{
  fr_lio::HighRateImuSample sample;
  sample.timestamp = timestamp;
  sample.acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
  return sample;
}

}  // namespace

TEST(HighRateOdomHealthGate, UsesHysteresisAndPosteriorGatedRecovery)
{
  fr_lio::HighRateOdomHealthGate gate(0.18, 0.12, 0.40, 3);

  EXPECT_EQ(gate.update(0.17, 0), fr_lio::HighRateOdomHealth::Healthy);
  EXPECT_EQ(gate.update(0.19, 0), fr_lio::HighRateOdomHealth::Suspect);
  EXPECT_EQ(gate.update(0.15, 0), fr_lio::HighRateOdomHealth::Suspect);
  EXPECT_EQ(gate.update(0.11, 0), fr_lio::HighRateOdomHealth::Healthy);

  EXPECT_EQ(gate.update(0.41, 0), fr_lio::HighRateOdomHealth::StaleLidar);
  EXPECT_EQ(gate.update(0.05, 0), fr_lio::HighRateOdomHealth::StaleLidar);
  EXPECT_EQ(gate.update(0.05, 1), fr_lio::HighRateOdomHealth::StaleLidar);
  EXPECT_EQ(gate.update(0.05, 1), fr_lio::HighRateOdomHealth::StaleLidar);
  EXPECT_EQ(gate.update(0.05, 1), fr_lio::HighRateOdomHealth::Healthy);
}

TEST(AccelerationUnitReport, ReportsWithoutChangingRuntimeUnits)
{
  fr_lio::AccelerationUnitReport report(10.0);
  for (int index = 0; index <= 1000; ++index) {
    report.add(index * 0.01, Eigen::Vector3d(0.0, 0.0, 9.81));
  }
  EXPECT_TRUE(report.ready());
  EXPECT_NEAR(report.mean_norm(), 9.81, 1e-9);
  EXPECT_EQ(report.suggested_unit(), "mps2");
  EXPECT_DOUBLE_EQ(
    fr_lio::acceleration_scale(fr_lio::AccelerationUnit::Unconfirmed), 1.0);
  EXPECT_DOUBLE_EQ(
    fr_lio::acceleration_scale(fr_lio::AccelerationUnit::Gravity), 9.81);
}

TEST(AccelerationUnitReport, RejectsUnknownConfigurationValue)
{
  EXPECT_FALSE(fr_lio::parse_acceleration_unit("auto").has_value());
}

TEST(HighRateOdomPropagator, SequentialStationaryIntegrationStaysStill)
{
  fr_lio::HighRateOdomPropagator propagator;
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor()));

  std::optional<fr_lio::HighRateOdomResult> result;
  for (int index = 1; index <= 20; ++index) {
    result = propagator.add_imu(stationary_imu(index * 0.005));
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->publish);
  EXPECT_NEAR(result->state.position.norm(), 0.0, 1e-9);
  EXPECT_NEAR(result->state.velocity.norm(), 0.0, 1e-9);
  EXPECT_NEAR(result->state.timestamp, 0.1, 1e-12);
}

TEST(HighRateOdomPropagator, RejectsAccelerationSpikeWithoutVelocityImpulse)
{
  fr_lio::HighRateAccelFilterConfig filter;
  filter.window_size = 9;
  filter.min_samples = 5;
  filter.max_deviation_mps2 = 5.0;
  filter.max_norm_mps2 = 19.62;
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, filter);

  for (int index = 0; index <= 6; ++index) {
    propagator.add_imu(stationary_imu(index * 0.005));
  }
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.03)));

  auto spike = stationary_imu(0.035);
  spike.acceleration.z() = 39.24;
  const auto rejected = propagator.add_imu(spike);
  ASSERT_TRUE(rejected.has_value());
  EXPECT_TRUE(rejected->accel_spike_rejected);
  EXPECT_TRUE(rejected->accel_norm_limit_exceeded);
  EXPECT_TRUE(rejected->accel_deviation_limit_exceeded);
  EXPECT_EQ(rejected->accel_spike_rejection_count, 1U);
  EXPECT_EQ(rejected->consecutive_accel_spike_rejections, 1U);
  EXPECT_NEAR(rejected->state.velocity.norm(), 0.0, 1e-12);
  EXPECT_NEAR(rejected->state.position.norm(), 0.0, 1e-12);

  const auto recovered = propagator.add_imu(stationary_imu(0.04));
  ASSERT_TRUE(recovered.has_value());
  EXPECT_FALSE(recovered->accel_spike_rejected);
  EXPECT_EQ(recovered->accel_spike_rejection_count, 1U);
  EXPECT_EQ(recovered->consecutive_accel_spike_rejections, 0U);
  EXPECT_NEAR(recovered->state.velocity.norm(), 0.0, 1e-12);
}

TEST(HighRateOdomPropagator, KeepsMedianReferenceAfterSustainedSpikeBurst)
{
  fr_lio::HighRateAccelFilterConfig filter;
  filter.window_size = 9;
  filter.min_samples = 5;
  filter.max_deviation_mps2 = 5.0;
  filter.max_norm_mps2 = 19.62;
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, filter);

  for (int index = 0; index <= 8; ++index) {
    propagator.add_imu(stationary_imu(index * 0.005));
  }
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.04)));

  std::optional<fr_lio::HighRateOdomResult> result;
  for (int index = 1; index <= 5; ++index) {
    auto spike = stationary_imu(0.04 + index * 0.005);
    spike.acceleration.z() = 39.24;
    result = propagator.add_imu(spike);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->accel_spike_rejected);
  }

  // The valid sample must still be measured against the stationary median,
  // rather than a median polluted by the five rejected spikes.
  result = propagator.add_imu(stationary_imu(0.07));
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accel_spike_rejected);
  EXPECT_NEAR(result->state.velocity.norm(), 0.0, 1e-12);
}

TEST(HighRateOdomPropagator, ReplaysFilteredSampleAfterDelayedLidarCorrection)
{
  fr_lio::HighRateAccelFilterConfig filter;
  filter.window_size = 9;
  filter.min_samples = 5;
  filter.max_deviation_mps2 = 5.0;
  filter.max_norm_mps2 = 19.62;
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, filter);

  for (int index = 0; index <= 6; ++index) {
    propagator.add_imu(stationary_imu(index * 0.005));
  }
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.03)));
  auto spike = stationary_imu(0.035);
  spike.acceleration.z() = 39.24;
  ASSERT_TRUE(propagator.add_imu(spike).has_value());
  ASSERT_TRUE(propagator.add_imu(stationary_imu(0.04)).has_value());

  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.0325)));
  const auto replayed = propagator.current();
  ASSERT_TRUE(replayed.has_value());
  EXPECT_NEAR(replayed->state.timestamp, 0.04, 1e-12);
  EXPECT_NEAR(replayed->state.velocity.norm(), 0.0, 1e-12);
  EXPECT_NEAR(replayed->state.position.norm(), 0.0, 1e-12);
}

TEST(HighRateOdomPropagator, AcceptsSustainedAccelerationAfterMedianWindowMoves)
{
  fr_lio::HighRateAccelFilterConfig filter;
  filter.window_size = 9;
  filter.min_samples = 5;
  filter.max_deviation_mps2 = 5.0;
  filter.max_norm_mps2 = 50.0;
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, filter);

  for (int index = 0; index <= 8; ++index) {
    propagator.add_imu(stationary_imu(index * 0.005));
  }
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.04)));

  std::optional<fr_lio::HighRateOdomResult> result;
  for (int index = 1; index <= 6; ++index) {
    auto accelerated = stationary_imu(0.04 + index * 0.005);
    accelerated.acceleration.x() = 6.0;
    result = propagator.add_imu(accelerated);
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accel_spike_rejected);
  EXPECT_GT(result->state.velocity.x(), 0.0);
}

TEST(HighRateOdomPropagator, ReplaysImuAfterDelayedLidarCorrection)
{
  fr_lio::HighRateOdomPropagator propagator;
  for (int index = 0; index <= 20; ++index) {
    propagator.add_imu(stationary_imu(index * 0.005));
  }

  auto corrected = stationary_anchor(0.05);
  corrected.position.x() = 2.0;
  ASSERT_TRUE(propagator.reset_from_lidar(corrected));
  const auto current = propagator.current();
  ASSERT_TRUE(current.has_value());
  EXPECT_NEAR(current->state.timestamp, 0.1, 1e-12);
  EXPECT_NEAR(current->state.position.x(), 2.0, 1e-9);
  EXPECT_NEAR(current->lidar_anchor_age_s, 0.05, 1e-12);
}

TEST(HighRateOdomPropagator, SmoothsLidarCorrectionForContinuousOutput)
{
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, {}, 0.25);
  for (int index = 0; index <= 20; ++index) {
    propagator.add_imu(stationary_imu(index * 0.005));
  }

  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.0)));
  ASSERT_TRUE(propagator.current().has_value());
  auto corrected = stationary_anchor(0.05);
  corrected.position.x() = 2.0;
  ASSERT_TRUE(propagator.reset_from_lidar(corrected));
  auto continuous = propagator.current();
  ASSERT_TRUE(continuous.has_value());
  EXPECT_NEAR(continuous->state.position.x(), 0.0, 1e-9);
  EXPECT_NEAR(continuous->state.velocity.x(), 0.0, 1e-12);

  for (int index = 21; index <= 45; ++index) {
    continuous = propagator.add_imu(stationary_imu(index * 0.005));
  }
  ASSERT_TRUE(continuous.has_value());
  EXPECT_GT(continuous->state.position.x(), 0.0);
  EXPECT_LT(continuous->state.position.x(), 2.0);
  EXPECT_NEAR(continuous->state.velocity.x(), 0.0, 1e-12);

  for (int index = 46; index <= 70; ++index) {
    continuous = propagator.add_imu(stationary_imu(index * 0.005));
  }
  ASSERT_TRUE(continuous.has_value());
  EXPECT_NEAR(continuous->state.position.x(), 2.0, 1e-6);
}

TEST(HighRateOdomPropagator, CorrectionDiagnosticExposesPropagationAndSmoothingStages)
{
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, {}, 0.25);
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.0)));
  ASSERT_TRUE(propagator.add_imu(stationary_imu(0.1)).has_value());

  auto corrected = stationary_anchor(0.05);
  corrected.velocity.x() = 2.0;
  ASSERT_TRUE(propagator.reset_from_lidar(corrected));

  const auto diagnostic = propagator.take_last_correction_diagnostic();
  ASSERT_TRUE(diagnostic.has_value());
  EXPECT_TRUE(diagnostic->valid);
  EXPECT_TRUE(diagnostic->smoothing_enabled);
  EXPECT_NEAR(diagnostic->propagated_velocity.x(), 0.0, 1e-12);
  EXPECT_NEAR(diagnostic->anchor_posterior_velocity.x(), 2.0, 1e-12);
  EXPECT_NEAR(diagnostic->posterior_velocity.x(), 2.0, 1e-12);
  EXPECT_NEAR(diagnostic->output_velocity_before.x(), 0.0, 1e-12);
  EXPECT_NEAR(diagnostic->output_velocity_after.x(), 2.0, 1e-12);
  EXPECT_NEAR(diagnostic->correction_offset_velocity.x(), -2.0, 1e-12);

  const auto current = propagator.current();
  ASSERT_TRUE(current.has_value());
  EXPECT_NEAR(current->state.velocity.x(), 2.0, 1e-12);
}

TEST(HighRateOdomPropagator, FirstAnchorWithoutImuStillProducesFreshDiagnostic)
{
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, {}, 0.25);
  auto corrected = stationary_anchor(1.0);
  corrected.velocity.z() = -0.5;
  ASSERT_TRUE(propagator.reset_from_lidar(corrected));

  const auto diagnostic = propagator.take_last_correction_diagnostic();
  ASSERT_TRUE(diagnostic.has_value());
  EXPECT_TRUE(diagnostic->valid);
  EXPECT_FALSE(diagnostic->smoothing_enabled);
  EXPECT_NEAR(diagnostic->anchor_posterior_velocity.z(), -0.5, 1e-12);
  EXPECT_NEAR(diagnostic->posterior_velocity.z(), -0.5, 1e-12);
  EXPECT_FALSE(propagator.take_last_correction_diagnostic().has_value());
}

TEST(HighRateOdomPropagator, CorrectionSmoothingBypassPublishesPosteriorDirectly)
{
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, {}, 0.0);
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.0)));
  ASSERT_TRUE(propagator.add_imu(stationary_imu(0.1)).has_value());

  auto corrected = stationary_anchor(0.05);
  corrected.velocity.x() = 2.0;
  ASSERT_TRUE(propagator.reset_from_lidar(corrected));

  const auto current = propagator.current();
  ASSERT_TRUE(current.has_value());
  EXPECT_NEAR(current->state.velocity.x(), 2.0, 1e-12);
  const auto diagnostic = propagator.take_last_correction_diagnostic();
  ASSERT_TRUE(diagnostic.has_value());
  EXPECT_FALSE(diagnostic->smoothing_enabled);
  EXPECT_NEAR(diagnostic->output_velocity_after.x(), 2.0, 1e-12);
}

TEST(HighRateOdomPropagator, NewAnchorUsesLatestReplayVelocity)
{
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 5.0, {}, 0.25);
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.0)));
  ASSERT_TRUE(propagator.add_imu(stationary_imu(0.1)).has_value());

  auto first = stationary_anchor(0.05);
  first.position.x() = 1.0;
  ASSERT_TRUE(propagator.reset_from_lidar(first));
  ASSERT_TRUE(propagator.add_imu(stationary_imu(0.15)).has_value());
  const auto before_second = propagator.current();
  ASSERT_TRUE(before_second.has_value());

  auto second = stationary_anchor(0.10);
  second.position.x() = 2.0;
  second.velocity.x() = 3.0;
  ASSERT_TRUE(propagator.reset_from_lidar(second));
  const auto diagnostic = propagator.take_last_correction_diagnostic();
  ASSERT_TRUE(diagnostic.has_value());
  EXPECT_NEAR(diagnostic->output_velocity_before.x(), 0.0, 1e-12);
  EXPECT_NEAR(diagnostic->posterior_velocity.x(), 3.0, 1e-12);
  EXPECT_NEAR(diagnostic->output_velocity_after.x(), 3.0, 1e-12);

  const auto after_second = propagator.current();
  ASSERT_TRUE(after_second.has_value());
  EXPECT_NEAR(after_second->state.position.x(), before_second->state.position.x(), 1e-12);
  EXPECT_NEAR(after_second->state.velocity.x(), 3.0, 1e-12);
}

TEST(HighRateOdomPropagator, InterpolatesInputAtDelayedCorrectionTime)
{
  fr_lio::HighRateOdomPropagator propagator;
  auto before = stationary_imu(0.0);
  before.acceleration.x() = 0.0;
  auto after = stationary_imu(0.1);
  after.acceleration.x() = 2.0;
  propagator.add_imu(before);
  propagator.add_imu(after);

  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.05)));
  const auto current = propagator.current();
  ASSERT_TRUE(current.has_value());
  EXPECT_NEAR(current->state.velocity.x(), 0.075, 1e-9);
  EXPECT_NEAR(current->state.position.x(), 0.001875, 1e-9);
}

TEST(HighRateOdomPropagator, ConstantAccelerationUsesEveryImuInterval)
{
  fr_lio::HighRateOdomPropagator propagator;
  auto first = stationary_imu(0.0);
  first.acceleration.x() = 1.0;
  propagator.add_imu(first);
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor()));

  std::optional<fr_lio::HighRateOdomResult> result;
  for (int index = 1; index <= 20; ++index) {
    auto sample = stationary_imu(index * 0.005);
    sample.acceleration.x() = 1.0;
    result = propagator.add_imu(sample);
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->state.velocity.x(), 0.1, 1e-6);
  EXPECT_NEAR(result->state.position.x(), 0.005, 1e-6);
}

TEST(HighRateOdomPropagator, IntegratesBiasCorrectedAngularVelocity)
{
  fr_lio::HighRateOdomPropagator propagator;
  auto initial = stationary_imu(0.0);
  initial.angular_velocity.z() = 1.1;
  propagator.add_imu(initial);
  auto anchor = stationary_anchor();
  anchor.gyro_bias.z() = 0.1;
  ASSERT_TRUE(propagator.reset_from_lidar(anchor));

  auto next = stationary_imu(0.1);
  next.angular_velocity.z() = 1.1;
  const auto result = propagator.add_imu(next);
  ASSERT_TRUE(result.has_value());
  const Eigen::AngleAxisd rotation(result->state.rotation);
  EXPECT_NEAR(rotation.angle(), 0.1, 1e-9);
  EXPECT_NEAR(rotation.axis().z(), 1.0, 1e-9);
  EXPECT_NEAR(result->body_angular_velocity.z(), 1.0, 1e-9);
}

TEST(HighRateOdomPropagator, TimestampRollbackInvalidatesUntilNextLidarAnchor)
{
  fr_lio::HighRateOdomPropagator propagator;
  propagator.add_imu(stationary_imu(1.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(1.0)));
  ASSERT_TRUE(propagator.add_imu(stationary_imu(1.01)).has_value());

  EXPECT_FALSE(propagator.add_imu(stationary_imu(0.5)).has_value());
  EXPECT_FALSE(propagator.current().has_value());
  EXPECT_FALSE(propagator.add_imu(stationary_imu(0.51)).has_value());

  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor(0.505)));
  EXPECT_TRUE(propagator.current().has_value());
}

TEST(HighRateOdomPropagator, RejectsLidarCorrectionFromFutureTimestamp)
{
  fr_lio::HighRateOdomPropagator propagator;
  propagator.add_imu(stationary_imu(1.0));
  EXPECT_FALSE(propagator.reset_from_lidar(stationary_anchor(1.01)));
  EXPECT_FALSE(propagator.current().has_value());
}

// A stale LiDAR posterior must NOT stop publication. It raises the level to
// PLANNER_UNUSABLE, which keeps EV usable and only stops the planner. Stopping
// the stream here is what produced the EV dropout cascade.
TEST(HighRateOdomPropagator, StaleLidarKeepsPublishingAndOnlyBlocksThePlanner)
{
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 2.0);
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor()));

  const auto suspect = propagator.add_imu(stationary_imu(0.20));
  ASSERT_TRUE(suspect.has_value());
  EXPECT_TRUE(suspect->publish);
  EXPECT_EQ(suspect->health, fr_lio::HighRateOdomHealth::Suspect);
  EXPECT_EQ(suspect->level, fr_lio::HighRateOdomLevel::Degraded);
  EXPECT_TRUE(fr_lio::ev_usable_at(suspect->level));
  EXPECT_TRUE(fr_lio::planner_usable_at(suspect->level));

  const auto stale = propagator.add_imu(stationary_imu(0.41));
  ASSERT_TRUE(stale.has_value());
  EXPECT_TRUE(stale->publish);
  EXPECT_EQ(stale->health, fr_lio::HighRateOdomHealth::StaleLidar);
  EXPECT_EQ(stale->level, fr_lio::HighRateOdomLevel::PlannerUnusable);
  EXPECT_TRUE(fr_lio::ev_usable_at(stale->level));
  EXPECT_FALSE(fr_lio::planner_usable_at(stale->level));
  EXPECT_EQ(stale->reason, fr_lio::HighRateOdomReason::AnchorStale);
  // The published state is the real propagated state at its real timestamp.
  EXPECT_DOUBLE_EQ(stale->state.timestamp, 0.41);
}

TEST(HighRateOdomPropagator, PredictorGenerationAdvancesOnEveryPropagation)
{
  fr_lio::HighRateOdomPropagator propagator({}, 0.15, 0.40, 2.0);
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor()));

  const auto first = propagator.add_imu(stationary_imu(0.01));
  ASSERT_TRUE(first.has_value());
  const auto second = propagator.add_imu(stationary_imu(0.02));
  ASSERT_TRUE(second.has_value());
  EXPECT_GT(second->predictor_generation, first->predictor_generation);
  EXPECT_TRUE(second->state_finite);
}

TEST(HighRateOdomHealthGate, AnchorAxisNeverReachesStateUnusable)
{
  fr_lio::HighRateOdomHealthGate gate(0.18, 0.12, 0.40, 3);
  // Even a multi-second anchor age is only ever PLANNER_UNUSABLE: the anchor
  // says nothing about whether the propagated state is still valid.
  EXPECT_EQ(gate.update_anchor(5.0, 0), fr_lio::HighRateOdomLevel::PlannerUnusable);
  EXPECT_TRUE(fr_lio::ev_usable_at(gate.anchorLevel()));
  EXPECT_FALSE(fr_lio::planner_usable_at(gate.anchorLevel()));
  EXPECT_EQ(gate.anchorReason(), fr_lio::HighRateOdomReason::AnchorStale);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(gate.update_anchor(nan, 0), fr_lio::HighRateOdomLevel::PlannerUnusable);
  EXPECT_TRUE(fr_lio::ev_usable_at(gate.anchorLevel()));
}

TEST(HighRateOdomHealthGate, AnchorLevelsFollowHysteresisAndPosteriorRecovery)
{
  fr_lio::HighRateOdomHealthGate gate(0.18, 0.12, 0.40, 3);

  EXPECT_EQ(gate.update_anchor(0.17, 0), fr_lio::HighRateOdomLevel::Good);
  EXPECT_EQ(gate.update_anchor(0.19, 0), fr_lio::HighRateOdomLevel::Degraded);
  EXPECT_EQ(gate.update_anchor(0.15, 0), fr_lio::HighRateOdomLevel::Degraded);
  EXPECT_EQ(gate.update_anchor(0.11, 0), fr_lio::HighRateOdomLevel::Good);

  EXPECT_EQ(gate.update_anchor(0.41, 0), fr_lio::HighRateOdomLevel::PlannerUnusable);
  // Age alone must not recover: a new posterior generation is required.
  EXPECT_EQ(gate.update_anchor(0.05, 0), fr_lio::HighRateOdomLevel::PlannerUnusable);
  EXPECT_EQ(gate.update_anchor(0.05, 1), fr_lio::HighRateOdomLevel::PlannerUnusable);
  EXPECT_EQ(gate.update_anchor(0.05, 1), fr_lio::HighRateOdomLevel::PlannerUnusable);
  EXPECT_EQ(gate.update_anchor(0.05, 1), fr_lio::HighRateOdomLevel::Good);
}

TEST(PredictorHealthGate, FreshPredictorStaysGoodRegardlessOfAnchor)
{
  fr_lio::PredictorHealthGate gate(0.15, 0.10, 3);
  fr_lio::PredictorHealthGate::Input input;
  input.predictor_age_s = 0.02;
  input.predictor_generation = 1;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::Good);
  EXPECT_TRUE(fr_lio::ev_usable_at(gate.level()));
}

TEST(PredictorHealthGate, StalePredictorTakesEvAwayAndLatchesUntilRecovery)
{
  fr_lio::PredictorHealthGate gate(0.15, 0.10, 3);
  fr_lio::PredictorHealthGate::Input input;
  input.predictor_age_s = 0.20;
  input.predictor_generation = 10;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::StateUnusable);
  EXPECT_FALSE(fr_lio::ev_usable_at(gate.level()));
  EXPECT_EQ(gate.reason(), fr_lio::HighRateOdomReason::PredictorStale);

  // A fresh age is not enough while the generation has not advanced past the
  // faulting one: a frozen predictor can report a small age forever.
  input.predictor_age_s = 0.02;
  input.predictor_generation = 10;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::StateUnusable);

  input.predictor_generation = 11;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::StateUnusable);
  input.predictor_generation = 12;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::StateUnusable);
  input.predictor_generation = 13;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::Good);
  EXPECT_TRUE(fr_lio::ev_usable_at(gate.level()));
}

TEST(PredictorHealthGate, RollbackNonFiniteAndUninitializedAreAllStateUnusable)
{
  fr_lio::PredictorHealthGate gate(0.15, 0.10, 3);
  fr_lio::PredictorHealthGate::Input input;
  input.predictor_age_s = 0.01;
  input.predictor_generation = 1;

  input.timestamp_monotonic = false;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::StateUnusable);
  EXPECT_EQ(gate.reason(), fr_lio::HighRateOdomReason::TimestampRollback);

  gate.reset();
  input.timestamp_monotonic = true;
  input.state_finite = false;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::StateUnusable);
  EXPECT_EQ(gate.reason(), fr_lio::HighRateOdomReason::NonFiniteState);

  gate.reset();
  input.state_finite = true;
  input.initialized = false;
  EXPECT_EQ(gate.update(input), fr_lio::HighRateOdomLevel::StateUnusable);
  EXPECT_EQ(gate.reason(), fr_lio::HighRateOdomReason::NotInitialized);
}

// This is the exact scenario from the requirement: anchor_age=0.8s with
// predictor_age=0.02s must yield ev_usable=true, planner_usable=false.
TEST(LocalizationHealth, StaleAnchorWithFreshPredictorKeepsEvUsable)
{
  fr_lio::HighRateOdomHealthGate anchor_gate(0.18, 0.12, 0.40, 3);
  fr_lio::PredictorHealthGate predictor_gate(0.15, 0.10, 3);

  const auto anchor_level = anchor_gate.update_anchor(0.80, 0);
  fr_lio::PredictorHealthGate::Input input;
  input.predictor_age_s = 0.02;
  input.predictor_generation = 5;
  const auto predictor_level = predictor_gate.update(input);

  fr_lio::LocalizationHealthSnapshot snapshot;
  snapshot.anchor_age_s = 0.80;
  snapshot.predictor_age_s = 0.02;
  fr_lio::merge_health_axes(
    snapshot, anchor_level, anchor_gate.anchorReason(),
    predictor_level, predictor_gate.reason());

  EXPECT_EQ(snapshot.level, fr_lio::HighRateOdomLevel::PlannerUnusable);
  EXPECT_TRUE(snapshot.ev_usable);
  EXPECT_FALSE(snapshot.planner_usable);
  EXPECT_EQ(snapshot.reason, fr_lio::HighRateOdomReason::AnchorStale);
}

TEST(LocalizationHealth, DeadPredictorOverridesAFreshAnchor)
{
  fr_lio::HighRateOdomHealthGate anchor_gate(0.18, 0.12, 0.40, 3);
  fr_lio::PredictorHealthGate predictor_gate(0.15, 0.10, 3);

  const auto anchor_level = anchor_gate.update_anchor(0.05, 1);
  fr_lio::PredictorHealthGate::Input input;
  input.predictor_age_s = 0.30;
  input.predictor_generation = 5;
  const auto predictor_level = predictor_gate.update(input);

  fr_lio::LocalizationHealthSnapshot snapshot;
  fr_lio::merge_health_axes(
    snapshot, anchor_level, anchor_gate.anchorReason(),
    predictor_level, predictor_gate.reason());

  EXPECT_EQ(snapshot.level, fr_lio::HighRateOdomLevel::StateUnusable);
  EXPECT_FALSE(snapshot.ev_usable);
  EXPECT_FALSE(snapshot.planner_usable);
  EXPECT_EQ(snapshot.reason, fr_lio::HighRateOdomReason::PredictorStale);
}

// A snapshot must be internally consistent: GOOD together with a large
// anchor_age is exactly the contradiction the single-snapshot rule forbids.
TEST(LocalizationHealth, LevelAndUsabilityNeverContradictEachOther)
{
  for (const auto level : {
      fr_lio::HighRateOdomLevel::Good, fr_lio::HighRateOdomLevel::Degraded,
      fr_lio::HighRateOdomLevel::PlannerUnusable,
      fr_lio::HighRateOdomLevel::StateUnusable})
  {
    fr_lio::LocalizationHealthSnapshot snapshot;
    fr_lio::merge_health_axes(
      snapshot, level, fr_lio::HighRateOdomReason::None,
      fr_lio::HighRateOdomLevel::Good, fr_lio::HighRateOdomReason::None);
    EXPECT_EQ(snapshot.ev_usable, level < fr_lio::HighRateOdomLevel::StateUnusable);
    EXPECT_EQ(snapshot.planner_usable, level < fr_lio::HighRateOdomLevel::PlannerUnusable);
    // planner_usable implies ev_usable: the planner can never be allowed to run
    // on a state that PX4 is not even allowed to fuse.
    if (snapshot.planner_usable) {
      EXPECT_TRUE(snapshot.ev_usable);
    }
  }
}

TEST(HighRateOdomPropagator, PublishedCovariancesAreFiniteAndPositive)
{
  fr_lio::HighRateOdomPropagator propagator;
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor()));
  const auto result = propagator.add_imu(stationary_imu(0.01));
  ASSERT_TRUE(result.has_value());

  const auto pose_covariance =
    fr_lio::HighRateOdomPropagator::pose_covariance(result->state);
  const auto twist_covariance = fr_lio::HighRateOdomPropagator::twist_covariance(
    result->state, result->body_angular_velocity, 0.1);
  EXPECT_TRUE(pose_covariance.allFinite());
  EXPECT_TRUE(twist_covariance.allFinite());
  EXPECT_GT(pose_covariance.diagonal().minCoeff(), 0.0);
  EXPECT_GT(twist_covariance.diagonal().minCoeff(), 0.0);
}

TEST(HighRateOdomPropagator, PoseAttitudeCovarianceIsExpressedInWorldFrame)
{
  auto state = stationary_anchor();
  state.rotation = Eigen::AngleAxisd(
    M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  state.covariance.setZero();
  state.covariance(3, 3) = 1.0;
  state.covariance(4, 4) = 2.0;
  state.covariance(5, 5) = 3.0;

  const auto covariance = fr_lio::HighRateOdomPropagator::pose_covariance(state);
  EXPECT_NEAR(covariance(3, 3), 2.0, 1e-12);
  EXPECT_NEAR(covariance(4, 4), 1.0, 1e-12);
  EXPECT_NEAR(covariance(5, 5), 3.0, 1e-12);
}

TEST(HighRateOdomPropagator, LongPropagationKeepsCovarianceSymmetricAndPositive)
{
  fr_lio::HighRateOdomPropagator propagator({}, 20.0, 30.0, 40.0);
  propagator.add_imu(stationary_imu(0.0));
  ASSERT_TRUE(propagator.reset_from_lidar(stationary_anchor()));

  std::optional<fr_lio::HighRateOdomResult> result;
  for (int index = 1; index <= 2000; ++index) {
    auto sample = stationary_imu(index * 0.005);
    sample.angular_velocity = Eigen::Vector3d(0.01, -0.02, 0.03);
    result = propagator.add_imu(sample);
  }
  ASSERT_TRUE(result.has_value());
  const auto & covariance = result->state.covariance;
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_NEAR((covariance - covariance.transpose()).norm(), 0.0, 1e-9);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 18, 18>> solver(covariance);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-9);
}

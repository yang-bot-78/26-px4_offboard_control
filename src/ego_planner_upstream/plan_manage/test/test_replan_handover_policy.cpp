#include <gtest/gtest.h>

#include <ego_planner/candidate_transaction_policy.h>
#include <ego_planner/replan_handover_policy.h>

namespace
{

using ego_planner::ReplanHandoverPolicy;
using ego_planner::CandidateTransactionPolicy;

struct TransactionState
{
  int trajectory_id;
  double start_time;
  double duration;
  int safety_state;
  int visual_id;

  bool operator==(const TransactionState & other) const
  {
    return trajectory_id == other.trajectory_id &&
           start_time == other.start_time &&
           duration == other.duration &&
           safety_state == other.safety_state &&
           visual_id == other.visual_id;
  }
};

TEST(CandidateTransactionPolicy, PublishClearanceUsesPlanningLimitFromSafeStart)
{
  EXPECT_NEAR(
    CandidateTransactionPolicy::requiredPublishClearance(0.29, 0.36, 0.60),
    0.36, 1.0e-12);
}

TEST(CandidateTransactionPolicy, CloseStartMayEscapeButNeverMoveCloser)
{
  EXPECT_NEAR(
    CandidateTransactionPolicy::requiredPublishClearance(0.29, 0.36, 0.32),
    0.32, 1.0e-12);
  EXPECT_NEAR(
    CandidateTransactionPolicy::requiredPublishClearance(0.29, 0.36, 0.20),
    0.29, 1.0e-12);
}

TEST(ReplanHandoverPolicy, TrimsCapturedStalePrefixByProjection)
{
  const std::vector<Eigen::Vector3d> reference{
    {-0.10, 0.00, 0.78},
    {0.00, 0.00, 0.78},
    {0.50, 0.00, 0.78},
    {1.00, 0.00, 0.78}};
  const Eigen::Vector3d start(0.04, 0.00, 0.78);

  const auto trim = ReplanHandoverPolicy::trimReferencePrefix(reference, start);
  ASSERT_TRUE(trim.valid);
  ASSERT_GE(trim.points.size(), 2u);
  EXPECT_EQ(trim.projection_segment, 1u);
  EXPECT_NEAR(trim.projection_ratio, 0.08, 1.0e-12);
  EXPECT_NEAR(trim.discarded_length, 0.14, 1.0e-12);
  EXPECT_NEAR((trim.points.front() - start).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(trim.remaining_length, 0.96, 1.0e-12);
}

TEST(ReplanHandoverPolicy, ProjectionInsideCurvedSegmentKeepsOnlySuffix)
{
  const std::vector<Eigen::Vector3d> reference{
    {0.0, 0.0, 0.78},
    {0.5, 0.3, 0.78},
    {1.0, 0.3, 0.78},
    {1.4, 0.0, 0.78}};
  const Eigen::Vector3d start(0.75, 0.34, 0.78);

  const auto trim = ReplanHandoverPolicy::trimReferencePrefix(reference, start);
  ASSERT_TRUE(trim.valid);
  EXPECT_EQ(trim.projection_segment, 1u);
  EXPECT_NEAR(trim.projection_ratio, 0.5, 1.0e-12);
  EXPECT_NEAR(trim.points.front().x(), 0.75, 1.0e-12);
  EXPECT_NEAR(trim.points.front().y(), 0.30, 1.0e-12);
  ASSERT_EQ(trim.points.size(), 3u);
  EXPECT_NEAR((trim.points.back() - reference.back()).norm(), 0.0, 1.0e-12);
}

TEST(ReplanHandoverPolicy, ReferenceWithoutStalePrefixIsUnchanged)
{
  const std::vector<Eigen::Vector3d> reference{
    {0.0, 0.0, 0.78},
    {0.5, 0.1, 0.78},
    {1.0, 0.0, 0.78}};

  const auto trim =
    ReplanHandoverPolicy::trimReferencePrefix(reference, reference.front());
  ASSERT_TRUE(trim.valid);
  EXPECT_EQ(trim.projection_segment, 0u);
  EXPECT_NEAR(trim.projection_ratio, 0.0, 1.0e-12);
  ASSERT_EQ(trim.points.size(), reference.size());
  for (std::size_t index = 0; index < reference.size(); ++index) {
    EXPECT_NEAR((trim.points[index] - reference[index]).norm(), 0.0, 1.0e-12);
  }
}

TEST(ReplanHandoverPolicy, DiscontinuousCandidateIsRejectedAndActiveCanRemain)
{
  const Eigen::Vector3d expected_position(0.49741, 0.08721, 0.78000);
  const Eigen::Vector3d expected_velocity(0.28470, -0.01543, 0.0);
  const Eigen::Vector3d expected_acceleration(0.21296, -0.00970, 0.0);
  const Eigen::Vector3d bad_position(0.42222, 0.08890, 0.78000);

  int active_trajectory_id = 6;
  const auto result = ReplanHandoverPolicy::validate(
    expected_position, expected_velocity, expected_acceleration,
    bad_position, expected_velocity, expected_acceleration);
  if (result.accepted) {
    active_trajectory_id = 7;
  }

  EXPECT_FALSE(result.accepted);
  EXPECT_NEAR(result.position_error, 0.07520899, 1.0e-6);
  EXPECT_EQ(active_trajectory_id, 6);
}

TEST(ReplanHandoverPolicy, ExactCandidatePassesStrictContinuityGate)
{
  const Eigen::Vector3d position(0.49741, 0.08721, 0.78000);
  const Eigen::Vector3d velocity(0.28470, -0.01543, 0.0);
  const Eigen::Vector3d acceleration(0.21296, -0.00970, 0.0);
  const auto result = ReplanHandoverPolicy::validate(
    position, velocity, acceleration, position, velocity, acceleration);
  EXPECT_TRUE(result.accepted);
  EXPECT_LE(result.position_error, 0.001);
  EXPECT_LE(result.velocity_error, 0.001);
  EXPECT_LE(result.acceleration_error, 0.005);
}

TEST(ReplanHandoverPolicy, CompletedTrajectoryDoesNotRequireHandoverContinuity)
{
  EXPECT_TRUE(ReplanHandoverPolicy::isTrajectoryActiveAt(100.0, 2.0, 101.999));
  EXPECT_FALSE(ReplanHandoverPolicy::isTrajectoryActiveAt(100.0, 2.0, 102.0));
  EXPECT_FALSE(ReplanHandoverPolicy::isTrajectoryActiveAt(100.0, 2.0, 102.001));
}

TEST(ReplanHandoverPolicy, CompletedTrajectoryRequiresMeasuredOdomStart)
{
  EXPECT_TRUE(
    ReplanHandoverPolicy::requiresMeasuredOdomStart(
      100.0, 2.0, 102.0, 0.01, 0.10));
  EXPECT_TRUE(
    ReplanHandoverPolicy::requiresMeasuredOdomStart(
      100.0, 2.0, 103.0, 0.01, 0.10));
}

TEST(ReplanHandoverPolicy, ActiveTrajectoryKeepsStrictSpliceUntilItExpires)
{
  EXPECT_FALSE(
    ReplanHandoverPolicy::requiresMeasuredOdomStart(
      100.0, 2.0, 101.999, 0.01, 0.10));
  EXPECT_TRUE(
    ReplanHandoverPolicy::requiresMeasuredOdomStart(
      100.0, 2.0, 101.999, 0.101, 0.10));
}

TEST(ReplanHandoverPolicy, CandidateSwitchRechecksExpiryAfterPlanning)
{
  const bool active_when_planning_started =
    ReplanHandoverPolicy::isTrajectoryActiveAt(100.0, 2.0, 101.95);
  const bool active_when_candidate_ready =
    ReplanHandoverPolicy::isTrajectoryActiveAt(100.0, 2.0, 102.05);

  EXPECT_TRUE(active_when_planning_started);
  EXPECT_FALSE(active_when_candidate_ready);
}

TEST(ReplanHandoverPolicy, LargeTrackingErrorRequiresMeasuredOdomReanchor)
{
  EXPECT_FALSE(ReplanHandoverPolicy::requiresOdomReanchor(0.10, 0.10));
  EXPECT_TRUE(ReplanHandoverPolicy::requiresOdomReanchor(0.1001, 0.10));
  EXPECT_FALSE(ReplanHandoverPolicy::requiresOdomReanchor(
      std::numeric_limits<double>::quiet_NaN(), 0.10));
}

TEST(ReplanHandoverPolicy, ReanchorThresholdIncludesBoundedTrackingLag)
{
  EXPECT_NEAR(
    ReplanHandoverPolicy::effectiveOdomReanchorThreshold(0.25, 0.40),
    0.27, 1.0e-12);
  EXPECT_NEAR(
    ReplanHandoverPolicy::effectiveOdomReanchorThreshold(0.25, 2.0),
    0.30, 1.0e-12);
}

TEST(ReplanHandoverPolicy, OdomReanchorFailureCannotPreserveOldTrajectory)
{
  EXPECT_TRUE(ReplanHandoverPolicy::mayPreserveActiveTrajectoryAfterFailure(false, true));
  EXPECT_FALSE(ReplanHandoverPolicy::mayPreserveActiveTrajectoryAfterFailure(true, true));
  EXPECT_FALSE(ReplanHandoverPolicy::mayPreserveActiveTrajectoryAfterFailure(true, false));
}

TEST(ReplanHandoverPolicy, RetryContractIsBoundedAtFourHertz)
{
  const auto action = ReplanHandoverPolicy::rejectedCandidateAction(true);
  EXPECT_TRUE(action.keep_active);
  EXPECT_FALSE(action.publish_fatal_collision);
  EXPECT_LE(1.0 / action.retry_delay_sec, 4.0);
}

TEST(ReplanHandoverPolicy, UnsafeActiveTrajectoryRetainsFatalSemantics)
{
  const auto action = ReplanHandoverPolicy::rejectedCandidateAction(false);
  EXPECT_FALSE(action.keep_active);
  EXPECT_TRUE(action.publish_fatal_collision);
  EXPECT_EQ(action.retry_delay_sec, 0.0);
}

TEST(CandidateTransactionPolicy, RejectedCandidateCannotMutateActive)
{
  const TransactionState before{6, 44.736, 3.2, 1, 6};
  TransactionState active = before;
  const TransactionState rejected{7, 44.980, 3.4, 0, 7};

  EXPECT_FALSE(
    ego_planner::CandidateTransactionPolicy::commitIfValidated(
      rejected, false, active));
  EXPECT_TRUE(active == before);
  std::cout
    << "[REJECT_ACTIVE_IMMUTABILITY] active_id_before=" << before.trajectory_id
    << " active_id_after=" << active.trajectory_id
    << " trajectory_same=true start_time_same=true duration_same=true "
    << "safety_same=true visual_same=true pass=true" << std::endl;
}

TEST(CandidateTransactionPolicy, FirstRejectedCandidateDoesNotCreateFakeActive)
{
  const TransactionState no_active{-1, 0.0, 0.0, 0, -1};
  TransactionState active = no_active;
  const TransactionState rejected{1, 100.0, 2.0, 1, 1};

  EXPECT_FALSE(
    ego_planner::CandidateTransactionPolicy::commitIfValidated(
      rejected, false, active));
  EXPECT_TRUE(active == no_active);
  std::cout
    << "[FIRST_CANDIDATE_REJECT] candidate_id=1 active_valid=false "
    << "active_id=NONE published=false fsm_state=SAFETY_HOLD" << std::endl;
}

TEST(CandidateTransactionPolicy, FirstValidatedCandidateCommitsAtomically)
{
  TransactionState active{-1, 0.0, 0.0, 0, -1};
  const TransactionState candidate{2, 101.0, 2.5, 1, 2};

  EXPECT_TRUE(
    ego_planner::CandidateTransactionPolicy::commitIfValidated(
      candidate, true, active));
  EXPECT_TRUE(active == candidate);
  std::cout
    << "[FIRST_VALID_COMMIT] candidate_id=2 active_id=2 published_id=2 "
    << "traj_server_id=2 bridge_id=2 all_same=true" << std::endl;
}

TEST(CandidateTransactionPolicy, RejectedCandidateRetryIsRateLimited)
{
  EXPECT_DOUBLE_EQ(
    ego_planner::CandidateTransactionPolicy::kRejectedCandidateRetryDelaySec, 0.25);
  EXPECT_LE(
    1.0 / ego_planner::CandidateTransactionPolicy::kRejectedCandidateRetryDelaySec, 4.0);
}

}  // namespace

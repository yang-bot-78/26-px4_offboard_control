#include <gtest/gtest.h>

#include <ego_planner/recovery_path_policy.h>

namespace
{

using ego_planner::RecoveryPathPolicy;

TEST(RecoveryPathPolicy, SamplesThreeTimesAcrossOneMapVoxel)
{
  EXPECT_NEAR(
    RecoveryPathPolicy::stableSampleSpacing(0.04, 0.15, 3),
    0.05, 1.0e-9);
  EXPECT_NEAR(
    RecoveryPathPolicy::stableSampleSpacing(0.06, 0.15, 3),
    0.06, 1.0e-9);
}

TEST(RecoveryPathPolicy, KeepsAFreeRollingGoal)
{
  const auto result = RecoveryPathPolicy::findForwardRejoin(
    {1.0, 2.0, 0.78}, {1.0, 0.0}, 1.2, 0.1, 3,
    true,
    [](const Eigen::Vector3d &) {return true;},
    [](const Eigen::Vector3d &) {return true;});

  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.forward_distance, 0.0, 1.0e-9);
  EXPECT_NEAR(result.point.x(), 1.0, 1.0e-9);
}

TEST(RecoveryPathPolicy, FailedNormalPlanAdvancesPastAFreeRollingGoal)
{
  const auto result = RecoveryPathPolicy::findForwardRejoin(
    {1.0, 2.0, 0.78}, {1.0, 0.0}, 1.2, 0.1, 3,
    false,
    [](const Eigen::Vector3d &) {return true;},
    [](const Eigen::Vector3d &) {return true;});

  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.forward_distance, 0.3, 1.0e-9);
  EXPECT_NEAR(result.point.x(), 1.3, 1.0e-9);
}

TEST(RecoveryPathPolicy, RequiresStableFreeSpaceBehindObstacle)
{
  const auto result = RecoveryPathPolicy::findForwardRejoin(
    {0.0, 0.0, 0.78}, {1.0, 0.0}, 1.2, 0.1, 3,
    true,
    [](const Eigen::Vector3d & point) {
      // A single free voxel at 0.4m is noise. Stable free space starts at 0.7m.
      return std::abs(point.x() - 0.4) < 1.0e-9 || point.x() >= 0.7;
    },
    [](const Eigen::Vector3d &) {return true;});

  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.forward_distance, 0.9, 1.0e-9);
  EXPECT_NEAR(result.point.x(), 0.9, 1.0e-9);
}

TEST(RecoveryPathPolicy, StopsAtSharedBoundary)
{
  const auto result = RecoveryPathPolicy::findForwardRejoin(
    {0.0, 0.0, 0.78}, {1.0, 0.0}, 1.2, 0.1, 3,
    true,
    [](const Eigen::Vector3d & point) {return point.x() >= 0.7;},
    [](const Eigen::Vector3d & point) {return point.x() <= 0.75;});

  EXPECT_FALSE(result.found);
}

TEST(RecoveryPathPolicy, ProjectsOntoTheActualPolyline)
{
  const std::vector<Eigen::Vector3d> reference{
    {0.0, 0.0, 0.78}, {0.5, 0.2, 0.78}, {1.0, 0.0, 0.78}};
  const auto projection = RecoveryPathPolicy::projectToPolyline(
    reference, {0.5, 0.2, 0.78});

  EXPECT_NEAR(projection.distance, 0.0, 1.0e-9);
  EXPECT_EQ(projection.segment, 0U);
}

TEST(RecoveryPathPolicy, RejoinFollowsPolylineAroundCorner)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.0, 0.0, 0.78}, {0.4, 0.0, 0.78}, {0.4, 0.6, 0.78}};
  const auto result = RecoveryPathPolicy::findForwardRejoinOnPolyline(
    continuation, 1.0, 0.1, 3,
    [](const Eigen::Vector3d & point) {return point.y() >= 0.2;},
    [](const Eigen::Vector3d &) {return true;});

  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.forward_distance, 0.8, 1.0e-9);
  EXPECT_NEAR(result.point.x(), 0.4, 1.0e-9);
  EXPECT_NEAR(result.point.y(), 0.4, 1.0e-9);
}

TEST(RecoveryPathPolicy, RejoinDoesNotExtrapolatePastPolylineEnd)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.0, 0.0, 0.78}, {0.3, 0.0, 0.78}};
  const auto result = RecoveryPathPolicy::findForwardRejoinOnPolyline(
    continuation, 1.0, 0.1, 3,
    [](const Eigen::Vector3d & point) {return point.x() >= 0.2;},
    [](const Eigen::Vector3d &) {return true;});

  EXPECT_FALSE(result.found);
}

// A 0.5m box centred on the reference line, inflated by a 0.40m planning
// clearance, blocks 1.3m of that line.  This is the C-area failure that
// produced 213 consecutive EGO_REJOIN_NOT_FOUND entries: an on-line search
// bounded by 0.60m cannot leave the blocked span.
namespace
{
constexpr double kPillarHalf = 0.25;
constexpr double kPillarClearance = 0.40;

bool freeAroundPillarAtOrigin(const Eigen::Vector3d & point)
{
  // Square obstacle centred at (1.0, 0.0) spanning +-0.25m, grown by the
  // recovery clearance.  Chebyshev distance matches a box, not a cylinder.
  const double dx = std::abs(point.x() - 1.0);
  const double dy = std::abs(point.y());
  return std::max(dx, dy) > kPillarHalf + kPillarClearance;
}
}  // namespace

TEST(RecoveryPathPolicy, OnLineSearchCannotClearAPillarOnTheReferenceLine)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.4, 0.0, 0.78}, {2.4, 0.0, 0.78}};
  const auto result = RecoveryPathPolicy::findForwardRejoinOnPolyline(
    continuation, 0.60, 0.05, 3, freeAroundPillarAtOrigin,
    [](const Eigen::Vector3d &) {return true;});

  // Documents the structural limit that motivates the lateral search.
  EXPECT_FALSE(result.found);
}

TEST(RecoveryPathPolicy, LateralSearchClearsAPillarOnTheReferenceLine)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.4, 0.0, 0.78}, {2.4, 0.0, 0.78}};
  const auto result = RecoveryPathPolicy::findLateralRejoinOnPolyline(
    continuation, 0.60, 0.05, 3,
    RecoveryPathPolicy::lateralOffsetLadder(0.80, 0.20),
    freeAroundPillarAtOrigin, [](const Eigen::Vector3d &) {return true;});

  ASSERT_TRUE(result.found);
  // The obstacle blocks |y| <= 0.65, so only the 0.80m rung escapes it.
  EXPECT_GT(std::abs(result.lateral_offset), kPillarHalf + kPillarClearance);
  EXPECT_TRUE(freeAroundPillarAtOrigin(result.point));
  EXPECT_LE(result.forward_distance, 0.60 + 1.0e-9);
  EXPECT_NEAR(result.point.z(), 0.78, 1.0e-9);
}

TEST(RecoveryPathPolicy, LateralSearchPrefersTheNearestForwardRejoin)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.0, 0.0, 0.78}, {2.0, 0.0, 0.78}};
  // Free only far ahead on the left, but close ahead on the right.  The nearest
  // forward distance must win even though the left offset is tried first.
  const auto is_free = [](const Eigen::Vector3d & point) {
      if (point.y() > 0.1) {return point.x() >= 1.5;}
      if (point.y() < -0.1) {return point.x() >= 0.3;}
      return false;
    };
  const auto result = RecoveryPathPolicy::findLateralRejoinOnPolyline(
    continuation, 2.0, 0.05, 3,
    RecoveryPathPolicy::lateralOffsetLadder(0.40, 0.20), is_free,
    [](const Eigen::Vector3d &) {return true;});

  ASSERT_TRUE(result.found);
  EXPECT_LT(result.lateral_offset, 0.0);
  EXPECT_LT(result.forward_distance, 1.5);
}

TEST(RecoveryPathPolicy, LateralSearchRespectsBoundsOnEveryOffsetLine)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.0, 0.0, 0.78}, {2.0, 0.0, 0.78}};
  // Everything is free, but the geofence forbids any lateral excursion, so the
  // offset lines are all rejected and no rejoin point may be reported.
  const auto result = RecoveryPathPolicy::findLateralRejoinOnPolyline(
    continuation, 2.0, 0.05, 3,
    RecoveryPathPolicy::lateralOffsetLadder(0.40, 0.20),
    [](const Eigen::Vector3d &) {return true;},
    [](const Eigen::Vector3d & point) {return std::abs(point.y()) <= 0.05;});

  EXPECT_FALSE(result.found);
}

TEST(RecoveryPathPolicy, AllLateralRejoinsAreOrderedByForwardDistance)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.4, 0.0, 0.78}, {2.4, 0.0, 0.78}};
  const auto all = RecoveryPathPolicy::findAllLateralRejoinsOnPolyline(
    continuation, 0.60, 0.05, 3,
    RecoveryPathPolicy::lateralOffsetLadder(0.80, 0.20),
    freeAroundPillarAtOrigin, [](const Eigen::Vector3d &) {return true;});

  // The pillar blocks |y| <= 0.65, so only the +-0.80 rungs escape it: the
  // caller gets both sides as alternatives rather than just the single best.
  ASSERT_GE(all.size(), 2U);
  for (std::size_t i = 1; i < all.size(); ++i) {
    EXPECT_LE(all[i - 1].forward_distance, all[i].forward_distance);
  }
  for (const auto & entry : all) {
    EXPECT_TRUE(entry.found);
    EXPECT_TRUE(freeAroundPillarAtOrigin(entry.point));
    EXPECT_GT(std::abs(entry.lateral_offset), kPillarHalf + kPillarClearance);
  }
  // Both sides must be represented so an unroutable one can be swapped out.
  bool has_left = false, has_right = false;
  for (const auto & entry : all) {
    if (entry.lateral_offset > 0.0) {has_left = true;}
    if (entry.lateral_offset < 0.0) {has_right = true;}
  }
  EXPECT_TRUE(has_left);
  EXPECT_TRUE(has_right);
}

TEST(RecoveryPathPolicy, AllLateralRejoinsAgreeWithTheSingleBest)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.0, 0.0, 0.78}, {2.0, 0.0, 0.78}};
  const auto is_free = [](const Eigen::Vector3d & point) {
      if (point.y() > 0.1) {return point.x() >= 1.5;}
      if (point.y() < -0.1) {return point.x() >= 0.3;}
      return false;
    };
  const auto ladder = RecoveryPathPolicy::lateralOffsetLadder(0.40, 0.20);
  const auto best = RecoveryPathPolicy::findLateralRejoinOnPolyline(
    continuation, 2.0, 0.05, 3, ladder, is_free,
    [](const Eigen::Vector3d &) {return true;});
  const auto all = RecoveryPathPolicy::findAllLateralRejoinsOnPolyline(
    continuation, 2.0, 0.05, 3, ladder, is_free,
    [](const Eigen::Vector3d &) {return true;});

  ASSERT_TRUE(best.found);
  ASSERT_FALSE(all.empty());
  // The head of the ordered list is exactly what the single-best call returns.
  EXPECT_NEAR(all.front().forward_distance, best.forward_distance, 1.0e-9);
  EXPECT_NEAR(all.front().lateral_offset, best.lateral_offset, 1.0e-9);
}

TEST(RecoveryPathPolicy, AllLateralRejoinsRejectsInvalidInput)
{
  const std::vector<Eigen::Vector3d> continuation{
    {0.0, 0.0, 0.78}, {2.0, 0.0, 0.78}};
  const auto always_free = [](const Eigen::Vector3d &) {return true;};
  EXPECT_TRUE(
    RecoveryPathPolicy::findAllLateralRejoinsOnPolyline(
      continuation, 2.0, 0.05, 3, {}, always_free, always_free).empty());
  EXPECT_TRUE(
    RecoveryPathPolicy::findAllLateralRejoinsOnPolyline(
      {}, 2.0, 0.05, 3, {0.2, -0.2}, always_free, always_free).empty());
  EXPECT_TRUE(
    RecoveryPathPolicy::findAllLateralRejoinsOnPolyline(
      continuation, 0.0, 0.05, 3, {0.2, -0.2}, always_free, always_free).empty());
}

TEST(RecoveryPathPolicy, LateralOffsetLadderIsSymmetricAndOrdered)
{
  const auto ladder = RecoveryPathPolicy::lateralOffsetLadder(0.60, 0.20);

  ASSERT_EQ(ladder.size(), 6U);
  EXPECT_NEAR(ladder[0], 0.20, 1.0e-9);
  EXPECT_NEAR(ladder[1], -0.20, 1.0e-9);
  EXPECT_NEAR(ladder[4], 0.60, 1.0e-9);
  EXPECT_NEAR(ladder[5], -0.60, 1.0e-9);
  EXPECT_TRUE(RecoveryPathPolicy::lateralOffsetLadder(0.0, 0.20).empty());
  EXPECT_TRUE(RecoveryPathPolicy::lateralOffsetLadder(0.60, 0.0).empty());
}

TEST(RecoveryPathPolicy, OffsetPolylineKeepsHeightAndStaysParallel)
{
  const std::vector<Eigen::Vector3d> reference{
    {0.0, 0.0, 0.78}, {1.0, 0.0, 0.78}};
  const auto left = RecoveryPathPolicy::offsetPolyline(reference, 0.30);

  ASSERT_EQ(left.size(), 2U);
  // Travel is +x, so the left normal is +y.
  EXPECT_NEAR(left[0].y(), 0.30, 1.0e-9);
  EXPECT_NEAR(left[1].y(), 0.30, 1.0e-9);
  EXPECT_NEAR(left[0].z(), 0.78, 1.0e-9);
  EXPECT_TRUE(RecoveryPathPolicy::offsetPolyline(reference, 0.0) == reference);
}

}  // namespace

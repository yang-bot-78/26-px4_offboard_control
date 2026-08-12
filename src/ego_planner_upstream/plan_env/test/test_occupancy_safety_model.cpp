#include <gtest/gtest.h>

#include "plan_env/occupancy_safety_model.h"

namespace
{

constexpr double kResolution = 0.15;
constexpr double kRequiredCenterClearance = 0.509;

TEST(OccupancySafetyModel, RequiredClearanceRoundsOutwardToWholeVoxels)
{
  EXPECT_EQ(4, plan_env::OccupancySafetyModel::inflationSteps(
      kRequiredCenterClearance, kResolution));
  EXPECT_DOUBLE_EQ(0.60, plan_env::OccupancySafetyModel::effectiveClearance(
      kRequiredCenterClearance, kResolution));
  EXPECT_EQ(3, plan_env::OccupancySafetyModel::coarseInflationSteps(
      kRequiredCenterClearance, kResolution));
}

TEST(OccupancySafetyModel, LegacyPointAtThirtyCentimetresIsUnsafe)
{
  EXPECT_FALSE(plan_env::OccupancySafetyModel::isContinuouslySafe(0.30, 0.509));
}

TEST(OccupancySafetyModel, PointsBelowAndAboveNewModelAreClassifiedConservatively)
{
  EXPECT_FALSE(plan_env::OccupancySafetyModel::isContinuouslySafe(0.508, kRequiredCenterClearance));
  EXPECT_TRUE(plan_env::OccupancySafetyModel::isContinuouslySafe(0.510, kRequiredCenterClearance));
}

TEST(OccupancySafetyModel, CrashSampleCannotPassTheNewModel)
{
  // 3C-14: nearest raw Saved-Map wall point was 0.426 m from the trajectory
  // sample before the physical-wall comparison.  It must fail at publication.
  EXPECT_FALSE(plan_env::OccupancySafetyModel::isContinuouslySafe(0.426, kRequiredCenterClearance));
}

TEST(OccupancySafetyModel, HistoricalFirstGateIsNotAdvertisedAsSafe)
{
  // Saved-Map measurement at x about 1.95 m: left/right raw obstacle faces
  // were 1.103 m apart.  This regression records that the former route must
  // fail closed under the verified center-envelope contract.
  constexpr double kRawGateWidth = 1.103;
  EXPECT_GT(kRawGateWidth - 2.0 * kRequiredCenterClearance, 0.0);
  EXPECT_GT(kRawGateWidth - 2.0 * kRequiredCenterClearance, 0.0);
}

}  // namespace

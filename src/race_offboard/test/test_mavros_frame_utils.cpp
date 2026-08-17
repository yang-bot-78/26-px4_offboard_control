#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "race_offboard/mavros_frame_utils.hpp"

namespace
{

constexpr double kEps = 1e-12;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

// Mirror of race_ego_bridge/frame_utils.hpp.  Duplicated on purpose: these
// tests must fail if the two conventions ever drift apart, which a shared
// include would hide.
std::array<double, 3> nedToMap(const std::array<double, 3> & ned)
{
  return {-ned[0], ned[1], -ned[2]};
}

double nedYawToMap(double yaw_ned)
{
  return race_offboard::wrapFrameAngle(M_PI - yaw_ned);
}

void expectVecNear(
  const std::array<double, 3> & actual, const std::array<double, 3> & expected)
{
  EXPECT_NEAR(actual[0], expected[0], kEps);
  EXPECT_NEAR(actual[1], expected[1], kEps);
  EXPECT_NEAR(actual[2], expected[2], kEps);
}

}  // namespace

TEST(MavrosFrameUtils, EnuToNedIsStandardAxisSwap)
{
  expectVecNear(race_offboard::enuToNed({1.0, 0.0, 0.0}), {0.0, 1.0, 0.0});
  expectVecNear(race_offboard::enuToNed({0.0, 1.0, 0.0}), {1.0, 0.0, 0.0});
  expectVecNear(race_offboard::enuToNed({0.0, 0.0, 1.0}), {0.0, 0.0, -1.0});
  expectVecNear(race_offboard::enuToNed({3.0, -2.0, 5.0}), {-2.0, 3.0, -5.0});
}

TEST(MavrosFrameUtils, PositionRoundTripIsIdentity)
{
  for (const auto & enu : {
    std::array<double, 3>{1.0, 2.0, 3.0},
    std::array<double, 3>{-4.0, 5.0, -6.0},
    std::array<double, 3>{0.1, -0.2, 0.3},
    std::array<double, 3>{0.0, 0.0, 0.0}})
  {
    expectVecNear(race_offboard::nedToEnu(race_offboard::enuToNed(enu)), enu);
    expectVecNear(race_offboard::enuToNed(race_offboard::nedToEnu(enu)), enu);
  }
}

TEST(MavrosFrameUtils, YawRoundTripSurvivesWrapAround)
{
  for (const double degrees :
    {0.0, 30.0, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0, -135.0, -180.0, 179.9, -179.9})
  {
    const double yaw_enu = degrees * M_PI / 180.0;
    const double round_trip =
      race_offboard::nedYawToEnu(race_offboard::enuYawToNed(yaw_enu));
    EXPECT_NEAR(race_offboard::wrapFrameAngle(round_trip - yaw_enu), 0.0, kEps)
      << "failed at " << degrees << " deg";
  }
}

TEST(MavrosFrameUtils, YawConversionMatchesCardinalDirections)
{
  // ENU yaw 0 points East; in NED that is 90 deg from North.
  EXPECT_NEAR(race_offboard::enuYawToNed(0.0), M_PI_2, kEps);
  // ENU yaw 90 deg points North, which is NED yaw 0.
  EXPECT_NEAR(race_offboard::enuYawToNed(M_PI_2), 0.0, kEps);
  // ENU yaw -90 deg points South, which is NED yaw 180.
  EXPECT_NEAR(
    std::fabs(race_offboard::enuYawToNed(-M_PI_2)), M_PI, kEps);
}

TEST(MavrosFrameUtils, MapToLocalAppliesTranslationOnlyToPosition)
{
  const race_offboard::PlanarFrameTransform transform{M_PI_2, 10.0, -2.0, 0.5};
  expectVecNear(
    race_offboard::mapToLocalPosition({2.0, 3.0, 1.0}, transform),
    {7.0, 0.0, 1.5});
  expectVecNear(
    race_offboard::mapToLocalVector({2.0, 3.0, 1.0}, transform),
    {-3.0, 2.0, 1.0});
  EXPECT_NEAR(race_offboard::mapToLocalYaw(-M_PI_2, transform), 0.0, kEps);
}

TEST(MavrosFrameUtils, MapGoalProjectsToTheSameLocalNedFrameAsVehiclePose)
{
  const race_offboard::PlanarFrameTransform transform{M_PI_2, 10.0, -2.0, 0.5};
  const auto goal_local_ned = race_offboard::enuToNed(
    race_offboard::mapToLocalPosition({2.0, 3.0, 1.0}, transform));
  expectVecNear(goal_local_ned, {0.0, 7.0, -1.5});
}

// The whole port rests on "px4_ned is standard NED".  If that is wrong, the
// planner's calibrated geofence is being interpreted in the wrong frame.
TEST(MavrosFrameUtils, EnuToProjectMapAgreesWithEgoBridgeConvention)
{
  for (const auto & enu : {
    std::array<double, 3>{1.0, 0.0, 0.0},
    std::array<double, 3>{0.0, 1.0, 0.0},
    std::array<double, 3>{0.0, 0.0, 1.0},
    std::array<double, 3>{3.0, -2.0, 5.0},
    std::array<double, 3>{-1.5, 0.7, -2.2}})
  {
    // Direct ENU -> map, as race_ego_bridge::enuToMap computes it.
    const std::array<double, 3> direct{-enu[1], enu[0], enu[2]};
    // Via this header's NED, then the ego bridge's nedToMap.
    const auto via_ned = nedToMap(race_offboard::enuToNed(enu));
    expectVecNear(via_ned, direct);
  }
}

TEST(MavrosFrameUtils, EnuYawToProjectMapAgreesWithEgoBridgeConvention)
{
  for (const double degrees : {0.0, 30.0, 45.0, 90.0, 135.0, 180.0, -45.0, -90.0, 179.0}) {
    const double yaw_enu = degrees * M_PI / 180.0;
    const double direct = race_offboard::wrapFrameAngle(yaw_enu + M_PI_2);
    const double via_ned = nedYawToMap(race_offboard::enuYawToNed(yaw_enu));
    EXPECT_NEAR(race_offboard::wrapFrameAngle(direct - via_ned), 0.0, kEps)
      << "failed at " << degrees << " deg";
  }
}

TEST(MavrosFrameUtils, TypeMaskIsZeroWhenEveryFeedForwardIsFinite)
{
  EXPECT_EQ(
    race_offboard::positionTargetTypeMask({0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}, true), 0u);
}

TEST(MavrosFrameUtils, TypeMaskIgnoresAllFeedForwardWhenAllNaN)
{
  const std::uint16_t mask =
    race_offboard::positionTargetTypeMask({kNaN, kNaN, kNaN}, {kNaN, kNaN, kNaN}, false);
  // 8|16|32|64|128|256|2048
  EXPECT_EQ(mask, 2552u);
}

TEST(MavrosFrameUtils, TypeMaskKeepsVelocityWhenOnlyAccelerationIsNaN)
{
  const std::uint16_t mask =
    race_offboard::positionTargetTypeMask({0.1, 0.2, 0.3}, {kNaN, kNaN, kNaN}, false);
  // 64|128|256|2048
  EXPECT_EQ(mask, 2496u);
}

// The over-height soft guard NaNs only the Z feed-forward.  A fixed mask would
// wrongly discard the XY feed-forward the local planner still needs.
TEST(MavrosFrameUtils, TypeMaskIgnoresOnlyTheZAxisForSoftOverheightGuard)
{
  const std::uint16_t mask =
    race_offboard::positionTargetTypeMask({0.5, -0.4, kNaN}, {0.2, 0.3, kNaN}, true);
  EXPECT_EQ(
    mask,
    race_offboard::PositionTargetMaskBits::kIgnoreVz |
    race_offboard::PositionTargetMaskBits::kIgnoreAfz);
  EXPECT_EQ(mask & race_offboard::PositionTargetMaskBits::kIgnoreVx, 0u);
  EXPECT_EQ(mask & race_offboard::PositionTargetMaskBits::kIgnoreVy, 0u);
  EXPECT_EQ(mask & race_offboard::PositionTargetMaskBits::kIgnoreAfx, 0u);
  EXPECT_EQ(mask & race_offboard::PositionTargetMaskBits::kIgnoreAfy, 0u);
  EXPECT_EQ(mask & race_offboard::PositionTargetMaskBits::kIgnoreYawRate, 0u);
}

TEST(MavrosFrameUtils, SanitizeReplacesNonFiniteAxesWithZero)
{
  expectVecNear(race_offboard::sanitizeIgnoredAxes({1.0, kNaN, 3.0}), {1.0, 0.0, 3.0});
  expectVecNear(
    race_offboard::sanitizeIgnoredAxes(
      {std::numeric_limits<double>::infinity(), kNaN, kNaN}), {0.0, 0.0, 0.0});
  expectVecNear(race_offboard::sanitizeIgnoredAxes({1.0, 2.0, 3.0}), {1.0, 2.0, 3.0});
}

#include <gtest/gtest.h>

#include "race_offboard/px4_local_pose_reset_matcher.hpp"

TEST(Px4LocalPoseResetMatcher, AcceptsTheNextFullXyzResetMatch)
{
  race_offboard::Px4LocalPoseResetMatcher matcher;
  matcher.configure(0.35, 0.10, 0.10);
  matcher.registerReset({0.18, -0.06, 0.54}, true, 1000000000);

  const auto result = matcher.observe({0.21, -0.10, 0.56}, 1030000000);
  EXPECT_EQ(result.decision, race_offboard::Px4LocalPoseResetDecision::Matched);
  EXPECT_TRUE(result.full_xy_metadata);
  EXPECT_NEAR(result.xy_residual_m, 0.05, 1e-12);
  EXPECT_NEAR(result.z_residual_m, 0.02, 1e-12);
}

TEST(Px4LocalPoseResetMatcher, RejectsZOnlyAuthorizationWithXyMotion)
{
  race_offboard::Px4LocalPoseResetMatcher matcher;
  matcher.configure(0.35, 0.10, 0.10);
  matcher.registerReset({0.0, 0.0, 0.54}, false, 1000000000);

  const auto result = matcher.observe({0.18, 0.02, 0.54}, 1030000000);
  EXPECT_EQ(result.decision, race_offboard::Px4LocalPoseResetDecision::Mismatched);
  EXPECT_FALSE(result.full_xy_metadata);
  EXPECT_GT(result.xy_residual_m, 0.10);
}

TEST(Px4LocalPoseResetMatcher, ExpiresWithoutAuthorizingALaterJump)
{
  race_offboard::Px4LocalPoseResetMatcher matcher;
  matcher.configure(0.25, 0.10, 0.10);
  matcher.registerReset({0.0, 0.0, 0.54}, true, 1000000000);

  EXPECT_EQ(
    matcher.observe({0.0, 0.0, 0.54}, 1300000000).decision,
    race_offboard::Px4LocalPoseResetDecision::Expired);
}

TEST(Px4LocalPoseResetMatcher, ConsumesTheAuthorizationAfterAMismatch)
{
  race_offboard::Px4LocalPoseResetMatcher matcher;
  matcher.configure(0.35, 0.10, 0.10);
  matcher.registerReset({0.0, 0.0, 0.54}, true, 1000000000);

  EXPECT_TRUE(matcher.pending());

  EXPECT_EQ(
    matcher.observe({0.0, 0.25, 0.54}, 1030000000).decision,
    race_offboard::Px4LocalPoseResetDecision::Mismatched);
  EXPECT_FALSE(matcher.pending());
  EXPECT_EQ(
    matcher.observe({0.0, 0.0, 0.54}, 1040000000).decision,
    race_offboard::Px4LocalPoseResetDecision::NoPending);
}

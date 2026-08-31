#include <gtest/gtest.h>

#include "race_offboard/mission_yaw_override_policy.hpp"

TEST(MissionYawOverridePolicy, AcceptsOnlyFreshCommandAtArmedOffboardGoalHold)
{
  EXPECT_TRUE(
    race_offboard::missionYawOverrideAllowed(
      true, true, true, true, true, true, 0.05, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, true, false, true, true, true, 0.05, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, true, true, false, true, true, 0.05, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, true, true, true, false, true, 0.05, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, true, true, true, true, false, 0.05, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, true, true, true, true, true, 0.31, 0.30));
}

TEST(MissionYawOverridePolicy, RejectsDisabledMissingOrInvalidTiming)
{
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      false, true, true, true, true, true, 0.05, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, false, true, true, true, true, 0.05, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, true, true, true, true, true, -0.01, 0.30));
  EXPECT_FALSE(
    race_offboard::missionYawOverrideAllowed(
      true, true, true, true, true, true, 0.05, 0.0));
}

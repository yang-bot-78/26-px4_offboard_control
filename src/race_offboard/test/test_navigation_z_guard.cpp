#include <gtest/gtest.h>

#include "race_offboard/navigation_z_guard.hpp"

TEST(NavigationZGuard, FirstCommandEstablishesReference)
{
  race_offboard::NavigationZGuard guard;
  guard.configure(0.08, 0.35);
  const auto result = guard.limit(-0.631, 1.0);
  EXPECT_FALSE(result.limited);
  EXPECT_DOUBLE_EQ(result.value, -0.631);
}

TEST(NavigationZGuard, LimitsFastHeightJumpByRate)
{
  race_offboard::NavigationZGuard guard;
  guard.configure(0.08, 0.35);
  guard.limit(-0.631, 1.0);
  const auto result = guard.limit(-0.875, 1.05);
  EXPECT_TRUE(result.limited);
  EXPECT_NEAR(result.value, -0.6485, 1.0e-12);
}

TEST(NavigationZGuard, LimitsLongGapByMaximumStep)
{
  race_offboard::NavigationZGuard guard;
  guard.configure(0.08, 0.35);
  guard.limit(-0.631, 1.0);
  const auto result = guard.limit(-0.300, 2.0);
  EXPECT_TRUE(result.limited);
  EXPECT_NEAR(result.value, -0.551, 1.0e-12);
}

TEST(NavigationZGuard, NeverMovesForNonMonotonicTimestamp)
{
  race_offboard::NavigationZGuard guard;
  guard.configure(0.08, 0.35);
  guard.limit(-0.631, 1.0);
  const auto result = guard.limit(-0.400, 0.9);
  EXPECT_TRUE(result.limited);
  EXPECT_NEAR(result.value, -0.631, 1.0e-12);
}

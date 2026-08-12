#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "race_offboard/local_origin_rebase_guard.hpp"

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

race_offboard::LocalOriginRebaseGuard makeGuard()
{
  race_offboard::LocalOriginRebaseGuard guard;
  guard.configure(1.0, 0.08, 0.20);
  return guard;
}

}  // namespace

TEST(GroundReferencePolicy, InitialLockRequiresDisarmAndReadyEvWhenConfigured)
{
  EXPECT_TRUE(race_offboard::initialGroundReferenceLockAllowed(false, true, true));
  EXPECT_FALSE(race_offboard::initialGroundReferenceLockAllowed(false, true, false));
  EXPECT_FALSE(race_offboard::initialGroundReferenceLockAllowed(true, true, true));
  EXPECT_TRUE(race_offboard::initialGroundReferenceLockAllowed(false, false, false));
  EXPECT_FALSE(race_offboard::initialGroundReferenceLockAllowed(true, false, true));
}

TEST(LocalOriginRebasePolicy, AllowsOnlyDisarmedIdleHealthyHold)
{
  EXPECT_TRUE(race_offboard::localOriginRebaseAllowed(false, true, true, true, true));
  EXPECT_FALSE(race_offboard::localOriginRebaseAllowed(true, true, true, true, true));
  EXPECT_FALSE(race_offboard::localOriginRebaseAllowed(false, false, true, true, true));
  EXPECT_FALSE(race_offboard::localOriginRebaseAllowed(false, true, false, true, true));
  EXPECT_FALSE(race_offboard::localOriginRebaseAllowed(false, true, true, false, true));
  EXPECT_FALSE(race_offboard::localOriginRebaseAllowed(false, true, true, true, false));
}

TEST(LocalOriginRebaseGuard, RequiresContinuousStableSamples)
{
  auto guard = makeGuard();
  const std::array<double, 3> reset_position{1.0, 2.0, 3.0};
  EXPECT_EQ(
    guard.observe(reset_position, 0.01, kSecond),
    race_offboard::LocalOriginRebaseDecision::CandidateStarted);
  EXPECT_TRUE(guard.active());
  EXPECT_EQ(
    guard.observe({1.01, 2.0, 3.0}, 0.02, kSecond + kSecond / 2),
    race_offboard::LocalOriginRebaseDecision::None);
  EXPECT_EQ(
    guard.observe({1.0, 2.01, 3.0}, 0.01, 2 * kSecond),
    race_offboard::LocalOriginRebaseDecision::Confirmed);
  EXPECT_FALSE(guard.active());
}

TEST(LocalOriginRebaseGuard, PositionSpreadRestartsConfirmation)
{
  auto guard = makeGuard();
  guard.observe({1.0, 2.0, 3.0}, 0.01, kSecond);
  EXPECT_EQ(
    guard.observe({1.20, 2.0, 3.0}, 0.01, kSecond + kSecond / 2),
    race_offboard::LocalOriginRebaseDecision::CandidateReset);
  EXPECT_EQ(
    guard.observe({1.20, 2.0, 3.0}, 0.01, 2 * kSecond),
    race_offboard::LocalOriginRebaseDecision::None);
  EXPECT_EQ(
    guard.observe({1.20, 2.0, 3.0}, 0.01, 2 * kSecond + kSecond / 2),
    race_offboard::LocalOriginRebaseDecision::Confirmed);
}

TEST(LocalOriginRebaseGuard, MotionCancelsCandidate)
{
  auto guard = makeGuard();
  guard.observe({1.0, 2.0, 3.0}, 0.01, kSecond);
  EXPECT_EQ(
    guard.observe({1.0, 2.0, 3.0}, 0.30, kSecond + kSecond / 2),
    race_offboard::LocalOriginRebaseDecision::None);
  EXPECT_FALSE(guard.active());
}

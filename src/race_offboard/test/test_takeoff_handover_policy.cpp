#include <gtest/gtest.h>

#include "race_offboard/takeoff_handover_policy.hpp"
#include "race_offboard/ego_command_guard.hpp"

TEST(ManualHandoverPolicy, TracksCurrentPositionBeforeOffboard)
{
  EXPECT_TRUE(race_offboard::shouldTrackManualHandoverPosition(true, true, true, false, true));
  EXPECT_FALSE(race_offboard::shouldTrackManualHandoverPosition(true, true, true, true, true));
  EXPECT_FALSE(race_offboard::shouldTrackManualHandoverPosition(true, true, false, false, true));
}

TEST(ManualHandoverPolicy, AcceptsOnlyAlignedSafeOffboardTransition)
{
  EXPECT_TRUE(
    race_offboard::canAcceptManualHandover(
      true, true, true, true, true, true, true, true, true));
  EXPECT_FALSE(
    race_offboard::canAcceptManualHandover(
      true, true, true, true, true, true, false, true, true));
  EXPECT_FALSE(
    race_offboard::canAcceptManualHandover(
      true, true, true, true, true, false, true, true, true));
  EXPECT_FALSE(
    race_offboard::canAcceptManualHandover(
      true, true, true, true, true, true, true, false, true));
  EXPECT_FALSE(
    race_offboard::canAcceptManualHandover(
      true, true, true, true, true, true, true, true, false));
}

TEST(ManualHandoverPolicy, RefinesReferenceOnlyDuringSafePositionHover)
{
  EXPECT_TRUE(race_offboard::canRefineManualHandoverReference(
    true, true, true, false, true, true, true, true, 0.10, 0.10));
  EXPECT_FALSE(race_offboard::canRefineManualHandoverReference(
    true, true, true, true, true, true, true, true, 0.01, 0.10));
  EXPECT_FALSE(race_offboard::canRefineManualHandoverReference(
    true, true, true, false, false, true, true, true, 0.01, 0.10));
  EXPECT_FALSE(race_offboard::canRefineManualHandoverReference(
    true, true, true, false, true, true, true, true, 0.11, 0.10));
  EXPECT_FALSE(race_offboard::canRefineManualHandoverReference(
    true, true, true, false, true, true, false, true, 0.01, 0.10));
}

TEST(PilotOverridePolicy, LeavingOffboardRevokesActiveAutomation)
{
  EXPECT_TRUE(race_offboard::pilotOverrideRequested(true, true, false, true, true));
  EXPECT_FALSE(race_offboard::pilotOverrideRequested(false, true, false, true, true));
  EXPECT_FALSE(race_offboard::pilotOverrideRequested(true, false, false, false, false));
}

TEST(PilotOverridePolicy, DisarmRevokesActiveAutomation)
{
  EXPECT_TRUE(race_offboard::pilotOverrideRequested(true, true, true, true, false));
  EXPECT_FALSE(race_offboard::pilotOverrideRequested(true, true, true, true, true));
}

TEST(AltitudeHoldPolicy, PlannerStopsKeepConfiguredFlightLevel)
{
  EXPECT_DOUBLE_EQ(-0.75, race_offboard::fixedAltitudeHoldZ(-0.75));
}

TEST(TakeoffHandoverPolicy, GroundGoalCanStartVerticalTakeoffWithoutEgoTrajectory)
{
  EXPECT_TRUE(race_offboard::canStartIndependentTakeoff(false));
  EXPECT_FALSE(race_offboard::canHandoverToEgo(false, false, false));
}

TEST(EgoCommandGuard, RejectsImpossiblePoseAndSetpointJumps)
{
  EXPECT_TRUE(race_offboard::localPositionJumpIsPlausible(0.55, 0.05, 4.0, 0.40));
  EXPECT_FALSE(race_offboard::localPositionJumpIsPlausible(10.0, 0.05, 4.0, 0.40));
  EXPECT_TRUE(race_offboard::egoSetpointIsNearTrustedPosition(1.2, 2.0));
  EXPECT_FALSE(race_offboard::egoSetpointIsNearTrustedPosition(10.0, 2.0));
}

TEST(EgoCommandGuard, RequiresAStableInitialPositionBeforeTakeoffLock)
{
  EXPECT_TRUE(race_offboard::initialPositionSampleIsStable(0.06, 0.10, 0.08, 0.20));
  EXPECT_FALSE(race_offboard::initialPositionSampleIsStable(0.09, 0.10, 0.08, 0.20));
  EXPECT_FALSE(race_offboard::initialPositionSampleIsStable(0.01, 0.21, 0.08, 0.20));
}

TEST(EgoCommandGuard, SlewsYawAcrossTheShortestDirection)
{
  constexpr double kPi = 3.14159265358979323846;
  EXPECT_NEAR(0.04, race_offboard::slewEgoYaw(0.0, kPi, 0.8, 0.05), 1e-6);
  EXPECT_NEAR(-kPi + 0.03, race_offboard::slewEgoYaw(kPi - 0.01, -kPi + 0.5, 0.8, 0.05), 1e-6);
}

TEST(TakeoffHandoverPolicy, HeightReachedStillHoldsWithoutFreshValidatedEgoTrajectory)
{
  EXPECT_FALSE(race_offboard::canHandoverToEgo(true, false, false));
  EXPECT_FALSE(race_offboard::canHandoverToEgo(true, true, true));
}

TEST(TakeoffHandoverPolicy, FreshSafeTrajectoryAllowsOnlyTheHandover)
{
  EXPECT_TRUE(race_offboard::canHandoverToEgo(true, true, false));
  EXPECT_FALSE(race_offboard::canStartIndependentTakeoff(true));
}

TEST(TakeoffHandoverPolicy, NewGoalWaitsForItsFirstEgoTrajectoryUntilExplicitFailure)
{
  EXPECT_TRUE(race_offboard::shouldHoldForInitialEgoTrajectory(true, false));
  EXPECT_TRUE(race_offboard::shouldHoldForInitialEgoTrajectory(true, false));
  EXPECT_FALSE(race_offboard::shouldHoldForInitialEgoTrajectory(true, true));
  EXPECT_FALSE(race_offboard::shouldHoldForInitialEgoTrajectory(false, false));
}

TEST(TakeoffHandoverPolicy, SlowInitialTrajectoryIsDiagnosticOnly)
{
  EXPECT_FALSE(race_offboard::initialEgoTrajectoryWaitIsSlow(1.999, 2.0));
  EXPECT_TRUE(race_offboard::initialEgoTrajectoryWaitIsSlow(2.0, 2.0));
  EXPECT_TRUE(race_offboard::initialEgoTrajectoryWaitIsSlow(10.0, 2.0));
}

TEST(TakeoffHandoverPolicy, EgoSetpointDoesNotRestartTakeoff)
{
  EXPECT_FALSE(race_offboard::shouldRestartTakeoffForEgoSetpoint());
}

TEST(TakeoffHandoverPolicy, StrictLocalPositionHealthIsExplicitlyRequested)
{
  EXPECT_FALSE(race_offboard::shouldRequireStrictLocalPositionHealth(false));
  EXPECT_TRUE(race_offboard::shouldRequireStrictLocalPositionHealth(true));
}

TEST(TakeoffHandoverPolicy, OnlyDistinctNonterminalFinalGoalMayResetPriorLatch)
{
  EXPECT_TRUE(race_offboard::canResetPlannerLatchForNewFinalGoal(true, false));
  EXPECT_FALSE(race_offboard::canResetPlannerLatchForNewFinalGoal(false, false));
  EXPECT_FALSE(race_offboard::canResetPlannerLatchForNewFinalGoal(true, true));
}

TEST(TakeoffHandoverPolicy, OnlyConfirmedEgoSafetyFailuresLatchPermanently)
{
  EXPECT_FALSE(
    race_offboard::egoPlannerStatusRequiresPermanentLatch(
      "EGO_TRAJECTORY_COLLISION"));
  EXPECT_TRUE(
    race_offboard::egoPlannerStatusRequiresPermanentLatch(
      "EGO_OCCUPANCY_STALE"));
  EXPECT_TRUE(
    race_offboard::egoPlannerStatusRequiresPermanentLatch(
      "EGO_REPLAN_REANCHOR_FAILED"));
  EXPECT_TRUE(race_offboard::egoPlannerStatusRequiresPermanentLatch("EMERGENCY_HOLD"));
  // A bridge report that nothing was trackable means no trajectory was cleared
  // for flight, but a later valid one recovers, so it must brake without
  // latching for the rest of the flight.
  EXPECT_FALSE(
    race_offboard::egoPlannerStatusRequiresPermanentLatch(
      "NO_TRACKABLE_POINT"));
}

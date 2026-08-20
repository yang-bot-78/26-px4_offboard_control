#include <gtest/gtest.h>

#include "ego_planner/recovery_failure_policy.h"

namespace ego_planner
{

TEST(RecoveryFailurePolicy, FirstAttemptIsImmediateAndCooldownDefersRetry)
{
  RecoveryFailurePolicy policy;
  policy.configure(2, 2.0, 5.0);
  policy.resetForNewGoal(17);

  EXPECT_EQ(policy.gate(100.0), RecoveryGateResult::ATTEMPT_NOW);
  policy.markAttempt(100.0);
  EXPECT_EQ(policy.gate(100.01), RecoveryGateResult::COOLDOWN_WAIT);
  EXPECT_NEAR(policy.cooldownRemaining(100.5), 1.5, 1e-9);
  EXPECT_EQ(policy.gate(102.0), RecoveryGateResult::ATTEMPT_NOW);
}

TEST(RecoveryFailurePolicy, FailureReportingIsDeduplicatedPerGoalAndReason)
{
  RecoveryFailurePolicy policy;
  policy.resetForNewGoal(4);
  EXPECT_TRUE(policy.shouldPublishFailure(4, "EGO_TRAJECTORY_COLLISION"));
  EXPECT_FALSE(policy.shouldPublishFailure(4, "EGO_TRAJECTORY_COLLISION"));
  EXPECT_TRUE(policy.shouldPublishFailure(5, "EGO_TRAJECTORY_COLLISION"));
  EXPECT_TRUE(policy.shouldPublishFailure(5, "EGO_OCCUPANCY_STALE"));
}

TEST(RecoveryFailurePolicy, ExhaustedAttemptsAreTerminalForTheGoal)
{
  RecoveryFailurePolicy policy;
  policy.configure(2, 0.5, 2.0);
  policy.resetForNewGoal(3);
  policy.markAttempt(1.0);
  policy.markAttempt(2.0);
  EXPECT_EQ(policy.gate(3.0), RecoveryGateResult::EXHAUSTED_TERMINAL);
  EXPECT_EQ(policy.gate(300.0), RecoveryGateResult::EXHAUSTED_TERMINAL);
  EXPECT_TRUE(policy.exhausted());
}

TEST(RecoveryFailurePolicy, ExhaustedGoalNeedsExplicitNewGoal)
{
  RecoveryFailurePolicy policy;
  policy.configure(2, 0.5, 2.0);
  policy.resetForNewGoal(3);
  policy.markAttempt(1.0);
  policy.markAttempt(2.0);

  EXPECT_EQ(policy.gate(3.0), RecoveryGateResult::EXHAUSTED_TERMINAL);
  policy.markAttempt(4.0);
  EXPECT_EQ(policy.attemptsForGoal(), 2);

  policy.resetForNewGoal(4);
  EXPECT_EQ(policy.gate(100.0), RecoveryGateResult::ATTEMPT_NOW);
  EXPECT_FALSE(policy.exhausted());
}

TEST(RecoveryFailurePolicy, MapUpdatesNeverResetAnExhaustedTransaction)
{
  RecoveryFailurePolicy policy;
  policy.configure(1, 0.5, 2.0);
  policy.resetForNewGoal(8);
  policy.markAttempt(1.0);
  policy.markLayoutUnsolvable(100);

  for (uint64_t revision = 101; revision < 1101; ++revision) {
    EXPECT_FALSE(policy.layoutKnownUnsolvable(revision));
    EXPECT_EQ(policy.gate(static_cast<double>(revision)), RecoveryGateResult::EXHAUSTED_TERMINAL);
  }
  EXPECT_EQ(policy.attemptsForGoal(), 1);
}

TEST(RecoveryFailurePolicy, SuccessfulCandidateClearsAttemptsForCurrentGoal)
{
  RecoveryFailurePolicy policy;
  policy.configure(2, 0.5, 2.0);
  policy.resetForNewGoal(7);
  policy.markAttempt(1.0);
  policy.markAttempt(2.0);
  policy.markSuccess();

  EXPECT_EQ(policy.attemptsForGoal(), 0);
  EXPECT_EQ(policy.gate(1.01), RecoveryGateResult::ATTEMPT_NOW);
}

TEST(RecoveryFailurePolicy, UnsolvableLayoutIsScopedToItsMapRevision)
{
  RecoveryFailurePolicy policy;
  policy.configure(2, 0.5, 2.0);
  policy.resetForNewGoal(11);

  EXPECT_FALSE(policy.layoutKnownUnsolvable(100));
  policy.markLayoutUnsolvable(100);
  EXPECT_TRUE(policy.layoutKnownUnsolvable(100));
  // A map update must re-enable the attempt: the layout that was unsolvable no
  // longer exists, so this can never park the vehicle permanently.
  EXPECT_FALSE(policy.layoutKnownUnsolvable(101));
}

TEST(RecoveryFailurePolicy, UnsolvableLayoutClearsOnNewGoalAndOnSuccess)
{
  RecoveryFailurePolicy policy;
  policy.configure(2, 0.5, 2.0);

  policy.resetForNewGoal(12);
  policy.markLayoutUnsolvable(200);
  policy.resetForNewGoal(13);
  EXPECT_FALSE(policy.layoutKnownUnsolvable(200));

  policy.markLayoutUnsolvable(200);
  policy.markSuccess();
  EXPECT_FALSE(policy.layoutKnownUnsolvable(200));

  policy.markLayoutUnsolvable(200);
  policy.clearLayoutUnsolvable();
  EXPECT_FALSE(policy.layoutKnownUnsolvable(200));
}

}  // namespace ego_planner

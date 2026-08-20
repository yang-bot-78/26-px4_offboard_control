#include <gtest/gtest.h>

#include "race_ego_bridge/handover_transaction_policy.hpp"

namespace race_ego_bridge
{

TEST(HandoverTransactionPolicy, SecondRejectionIsTerminal)
{
  EXPECT_FALSE(HandoverTransactionPolicy::exhausted(1, 2, 0.2, 1.5));
  EXPECT_TRUE(HandoverTransactionPolicy::exhausted(2, 2, 0.4, 1.5));
}

TEST(HandoverTransactionPolicy, DeadlineCannotBeRefreshedByAnotherCandidate)
{
  EXPECT_FALSE(HandoverTransactionPolicy::exhausted(1, 2, 1.49, 1.5));
  EXPECT_TRUE(HandoverTransactionPolicy::exhausted(1, 2, 1.50, 1.5));
}

TEST(HandoverTransactionPolicy, ActiveStoppingHorizonShortensDeadline)
{
  EXPECT_NEAR(
    HandoverTransactionPolicy::effectiveTimeout(1.5, 1.0, 0.4, 0.15, 0.8),
    0.20, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    HandoverTransactionPolicy::effectiveTimeout(1.5, 0.7, 0.4, 0.15, 0.8),
    0.0);
}

TEST(HandoverTransactionPolicy, LateCandidateCannotCommitAfterExhaustion)
{
  EXPECT_TRUE(HandoverTransactionPolicy::candidateAllowed(false));
  EXPECT_FALSE(HandoverTransactionPolicy::candidateAllowed(true));
}

}  // namespace race_ego_bridge

#ifndef EGO_PLANNER_CANDIDATE_TRANSACTION_POLICY_H_
#define EGO_PLANNER_CANDIDATE_TRANSACTION_POLICY_H_

#include <algorithm>

namespace ego_planner
{

  class CandidateTransactionPolicy
  {
public:
    static constexpr double kRejectedCandidateRetryDelaySec = 0.25;

    // A candidate normally has to preserve the planner's larger clearance.
    // If its handover point is already between the hard and planning limits,
    // requiring an instantaneous jump outward would make every escape
    // trajectory impossible. In that case, never allow it to get closer than
    // its start; the optimizer can then move the remaining suffix outward.
    static double requiredPublishClearance(
      const double hard_clearance, const double planning_clearance,
      const double start_clearance)
    {
      return std::max(
        hard_clearance,
        std::min(planning_clearance, start_clearance));
    }

    template < typename State >
    static bool commitIfValidated(
      const State & candidate, const bool all_validations_passed, State & active)
    {
      if (!all_validations_passed) {
        return false;
      }
      active = candidate;
      return true;
    }
  };

}  // namespace ego_planner

#endif  // EGO_PLANNER_CANDIDATE_TRANSACTION_POLICY_H_

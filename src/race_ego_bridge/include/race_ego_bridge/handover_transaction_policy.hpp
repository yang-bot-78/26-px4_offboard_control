#ifndef RACE_EGO_BRIDGE__HANDOVER_TRANSACTION_POLICY_HPP_
#define RACE_EGO_BRIDGE__HANDOVER_TRANSACTION_POLICY_HPP_

#include <algorithm>
#include <cmath>

namespace race_ego_bridge
{

class HandoverTransactionPolicy
{
public:
  static double effectiveTimeout(
    const double configured_timeout_sec, const double active_remaining_sec,
    const double speed_mps, const double reaction_time_sec,
    const double braking_deceleration_mps2, const double stop_margin_sec = 0.15)
  {
    if (!std::isfinite(configured_timeout_sec) || configured_timeout_sec <= 0.0 ||
      !std::isfinite(active_remaining_sec))
    {
      return 0.0;
    }
    const double speed = std::max(0.0, std::isfinite(speed_mps) ? speed_mps : 0.0);
    const double deceleration = std::max(
      0.10, std::isfinite(braking_deceleration_mps2) ? braking_deceleration_mps2 : 0.10);
    const double reaction = std::max(
      0.0, std::isfinite(reaction_time_sec) ? reaction_time_sec : 0.0);
    const double margin = std::max(
      0.0, std::isfinite(stop_margin_sec) ? stop_margin_sec : 0.0);
    const double stopping_time = reaction + speed / deceleration + margin;
    return std::max(
      0.0, std::min(configured_timeout_sec, active_remaining_sec - stopping_time));
  }

  static bool exhausted(
    const int rejection_count, const int max_rejections,
    const double transaction_age_sec, const double effective_timeout_sec)
  {
    return rejection_count >= std::max(1, max_rejections) ||
           !std::isfinite(transaction_age_sec) || transaction_age_sec < 0.0 ||
           transaction_age_sec >= std::max(0.0, effective_timeout_sec);
  }

  static bool candidateAllowed(const bool transaction_exhausted)
  {
    return !transaction_exhausted;
  }
};

}  // namespace race_ego_bridge

#endif  // RACE_EGO_BRIDGE__HANDOVER_TRANSACTION_POLICY_HPP_

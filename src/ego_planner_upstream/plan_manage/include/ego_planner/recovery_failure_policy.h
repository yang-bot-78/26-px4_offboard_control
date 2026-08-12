#ifndef EGO_PLANNER__RECOVERY_FAILURE_POLICY_H_
#define EGO_PLANNER__RECOVERY_FAILURE_POLICY_H_

#include <cstdint>
#include <string>

namespace ego_planner
{

enum class RecoveryGateResult
{
  ATTEMPT_NOW,
  COOLDOWN_WAIT,
  EXHAUSTED_BACKOFF_WAIT
};

// Keeps recovery rate limiting separate from safety-failure reporting.  A
// cooldown is an intentional deferred retry, never a failed recovery attempt.
class RecoveryFailurePolicy
{
public:
  void configure(int max_attempts, double cooldown_sec, double exhausted_backoff_sec)
  {
    max_attempts_ = max_attempts;
    cooldown_sec_ = cooldown_sec;
    exhausted_backoff_sec_ = exhausted_backoff_sec;
  }

  void resetForNewGoal(uint64_t goal_seq)
  {
    active_goal_seq_ = goal_seq;
    attempts_for_goal_ = 0;
    clearReportedFailure();
    clearLayoutUnsolvable();
  }

  void markSuccess()
  {
    attempts_for_goal_ = 0;
    has_last_attempt_ = false;
    clearReportedFailure();
    clearLayoutUnsolvable();
  }

  RecoveryGateResult gate(double now_sec) const
  {
    if (attempts_for_goal_ >= max_attempts_) {
      if (has_last_attempt_ && now_sec - last_attempt_sec_ < exhausted_backoff_sec_) {
        return RecoveryGateResult::EXHAUSTED_BACKOFF_WAIT;
      }
      return RecoveryGateResult::ATTEMPT_NOW;
    }
    if (has_last_attempt_ && now_sec - last_attempt_sec_ < cooldown_sec_) {
      return RecoveryGateResult::COOLDOWN_WAIT;
    }
    return RecoveryGateResult::ATTEMPT_NOW;
  }

  // Retrying with an unchanged goal *and* an unchanged obstacle layout re-runs an
  // identical computation and must reach an identical conclusion.  In the
  // 2026-08-06 C-area log that produced 1271 recovery attempts and 1146 completed
  // B-spline optimizations over 559s, all discarded.  Once a layout has been
  // shown to be unsolvable, wait for the map to actually change rather than
  // burning the optimizer on the same input.
  //
  // Deliberately not a permanent give-up: any new goal or any map update clears
  // it, and the exhausted backoff still schedules retries, so this cannot be the
  // reason the vehicle stays parked.
  void markLayoutUnsolvable(uint64_t map_revision)
  {
    unsolvable_layout_ = true;
    unsolvable_map_revision_ = map_revision;
  }

  bool layoutKnownUnsolvable(uint64_t map_revision) const
  {
    return unsolvable_layout_ && unsolvable_map_revision_ == map_revision;
  }

  void clearLayoutUnsolvable()
  {
    unsolvable_layout_ = false;
    unsolvable_map_revision_ = 0;
  }

  double cooldownRemaining(double now_sec) const
  {
    if (!has_last_attempt_) {
      return 0.0;
    }
    const double remaining = cooldown_sec_ - (now_sec - last_attempt_sec_);
    return remaining > 0.0 ? remaining : 0.0;
  }

  double exhaustedBackoffRemaining(double now_sec) const
  {
    if (!has_last_attempt_ || attempts_for_goal_ < max_attempts_) {
      return 0.0;
    }
    const double remaining = exhausted_backoff_sec_ - (now_sec - last_attempt_sec_);
    return remaining > 0.0 ? remaining : 0.0;
  }

  void markAttempt(double now_sec)
  {
    if (attempts_for_goal_ >= max_attempts_) {
      attempts_for_goal_ = 0;
    }
    ++attempts_for_goal_;
    last_attempt_sec_ = now_sec;
    has_last_attempt_ = true;
  }

  int attemptsForGoal() const { return attempts_for_goal_; }

  bool shouldPublishFailure(uint64_t goal_seq, const std::string & reason)
  {
    if (reported_failure_ && reported_goal_seq_ == goal_seq && reported_reason_ == reason) {
      return false;
    }
    reported_failure_ = true;
    reported_goal_seq_ = goal_seq;
    reported_reason_ = reason;
    return true;
  }

  void clearReportedFailure()
  {
    reported_failure_ = false;
    reported_goal_seq_ = 0;
    reported_reason_.clear();
  }

private:
  int max_attempts_{1};
  int attempts_for_goal_{0};
  double cooldown_sec_{0.0};
  double exhausted_backoff_sec_{2.0};
  double last_attempt_sec_{0.0};
  bool has_last_attempt_{false};
  uint64_t active_goal_seq_{0};
  bool unsolvable_layout_{false};
  uint64_t unsolvable_map_revision_{0};
  bool reported_failure_{false};
  uint64_t reported_goal_seq_{0};
  std::string reported_reason_;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__RECOVERY_FAILURE_POLICY_H_

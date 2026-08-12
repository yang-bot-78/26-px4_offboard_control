#ifndef PX4_ROS_COM__FRESH_RELAY_POLICY_HPP_
#define PX4_ROS_COM__FRESH_RELAY_POLICY_HPP_

#include <cstdint>

namespace px4_ros_com
{

enum class FreshnessDecision
{
  kPublish,
  kStale,
  kFuture,
  kInvalidStamp,
};

enum class ImuGapInjectionState
{
  kDisabled,
  kReady,
  kArmed,
  kCompleted,
  kAborted,
};

enum class ImuGapInjectionDecision
{
  kPublish,
  kDrop,
  kCompleteAndPublish,
  kAbortAndPublish,
};

class OneShotImuGapInjector
{
public:
  static constexpr std::int64_t kMinimumGapNs = 20000000LL;
  static constexpr std::int64_t kMaximumGapNs = 50000000LL;

  static bool target_is_safe(std::int64_t target_gap_ns)
  {
    return target_gap_ns > kMinimumGapNs && target_gap_ns < kMaximumGapNs;
  }

  bool configure(bool enabled, std::int64_t target_gap_ns)
  {
    if (enabled && !target_is_safe(target_gap_ns)) {
      return false;
    }
    enabled_ = enabled;
    target_gap_ns_ = target_gap_ns;
    state_ = enabled ? ImuGapInjectionState::kReady :
      ImuGapInjectionState::kDisabled;
    return true;
  }

  bool arm()
  {
    ++request_count_;
    if (!enabled_ || state_ != ImuGapInjectionState::kReady ||
      last_published_source_ns_ <= 0)
    {
      return false;
    }
    anchor_source_ns_ = last_published_source_ns_;
    completion_source_ns_ = 0;
    actual_gap_ns_ = 0;
    dropped_count_ = 0;
    state_ = ImuGapInjectionState::kArmed;
    return true;
  }

  ImuGapInjectionDecision observe_publishable_source(std::int64_t source_ns)
  {
    if (state_ == ImuGapInjectionState::kReady) {
      last_published_source_ns_ = source_ns;
      return ImuGapInjectionDecision::kPublish;
    }
    if (state_ != ImuGapInjectionState::kArmed) {
      last_published_source_ns_ = source_ns;
      return ImuGapInjectionDecision::kPublish;
    }
    if (source_ns <= anchor_source_ns_) {
      state_ = ImuGapInjectionState::kAborted;
      completion_source_ns_ = source_ns;
      actual_gap_ns_ = source_ns - anchor_source_ns_;
      last_published_source_ns_ = source_ns;
      return ImuGapInjectionDecision::kAbortAndPublish;
    }
    const std::int64_t gap_ns = source_ns - anchor_source_ns_;
    if (gap_ns < target_gap_ns_) {
      ++dropped_count_;
      return ImuGapInjectionDecision::kDrop;
    }
    completion_source_ns_ = source_ns;
    actual_gap_ns_ = gap_ns;
    last_published_source_ns_ = source_ns;
    if (gap_ns > kMinimumGapNs && gap_ns < kMaximumGapNs) {
      state_ = ImuGapInjectionState::kCompleted;
      return ImuGapInjectionDecision::kCompleteAndPublish;
    }
    state_ = ImuGapInjectionState::kAborted;
    return ImuGapInjectionDecision::kAbortAndPublish;
  }

  ImuGapInjectionState state() const {return state_;}
  std::int64_t target_gap_ns() const {return target_gap_ns_;}
  std::int64_t anchor_source_ns() const {return anchor_source_ns_;}
  std::int64_t completion_source_ns() const {return completion_source_ns_;}
  std::int64_t actual_gap_ns() const {return actual_gap_ns_;}
  std::uint64_t dropped_count() const {return dropped_count_;}
  std::uint64_t request_count() const {return request_count_;}

private:
  bool enabled_{false};
  ImuGapInjectionState state_{ImuGapInjectionState::kDisabled};
  std::int64_t target_gap_ns_{30000000LL};
  std::int64_t last_published_source_ns_{0};
  std::int64_t anchor_source_ns_{0};
  std::int64_t completion_source_ns_{0};
  std::int64_t actual_gap_ns_{0};
  std::uint64_t dropped_count_{0};
  std::uint64_t request_count_{0};
};

inline FreshnessDecision classify_header_age_ns(
  std::int64_t now_ns,
  std::int64_t stamp_ns,
  std::int64_t max_age_ns,
  std::int64_t max_future_ns)
{
  if (stamp_ns <= 0 || max_age_ns < 0 || max_future_ns < 0) {
    return FreshnessDecision::kInvalidStamp;
  }
  const std::int64_t age_ns = now_ns - stamp_ns;
  if (age_ns > max_age_ns) {
    return FreshnessDecision::kStale;
  }
  if (age_ns < -max_future_ns) {
    return FreshnessDecision::kFuture;
  }
  return FreshnessDecision::kPublish;
}

struct FreshRelayCounters
{
  std::uint64_t received{0};
  std::uint64_t stale_dropped{0};
  std::uint64_t future_dropped{0};
  std::uint64_t invalid_stamp_dropped{0};
  std::uint64_t injected_dropped{0};
  std::uint64_t published{0};
  std::int64_t first_header_age_ns{0};
  std::int64_t max_header_age_ns{0};
  bool header_age_observed{false};
  std::int64_t last_source_timestamp_ns{0};
  std::int64_t max_source_interarrival_ns{0};
  std::uint64_t source_gap_over_threshold{0};
  bool source_timestamp_observed{false};

  void observe(
    std::int64_t age_ns,
    bool age_valid,
    FreshnessDecision decision,
    std::int64_t source_timestamp_ns = 0,
    std::int64_t source_gap_threshold_ns = 0,
    bool publish_allowed = true)
  {
    ++received;
    if (age_valid) {
      if (!header_age_observed) {
        first_header_age_ns = age_ns;
        max_header_age_ns = age_ns;
        header_age_observed = true;
      } else if (age_ns > max_header_age_ns) {
        max_header_age_ns = age_ns;
      }
    }

    if (source_timestamp_ns > 0) {
      if (source_timestamp_observed && source_timestamp_ns > last_source_timestamp_ns) {
        const std::int64_t interarrival_ns = source_timestamp_ns - last_source_timestamp_ns;
        if (interarrival_ns > max_source_interarrival_ns) {
          max_source_interarrival_ns = interarrival_ns;
        }
        if (source_gap_threshold_ns > 0 && interarrival_ns > source_gap_threshold_ns) {
          ++source_gap_over_threshold;
        }
      }
      last_source_timestamp_ns = source_timestamp_ns;
      source_timestamp_observed = true;
    }

    switch (decision) {
      case FreshnessDecision::kPublish:
        if (publish_allowed) {
          ++published;
        } else {
          ++injected_dropped;
        }
        break;
      case FreshnessDecision::kStale:
        ++stale_dropped;
        break;
      case FreshnessDecision::kFuture:
        ++future_dropped;
        break;
      case FreshnessDecision::kInvalidStamp:
        ++invalid_stamp_dropped;
        break;
    }
  }
};

}  // namespace px4_ros_com

#endif  // PX4_ROS_COM__FRESH_RELAY_POLICY_HPP_

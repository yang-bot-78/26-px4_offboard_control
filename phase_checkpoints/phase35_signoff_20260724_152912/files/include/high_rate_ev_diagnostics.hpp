#ifndef FAST_LIO__HIGH_RATE_EV_DIAGNOSTICS_HPP_
#define FAST_LIO__HIGH_RATE_EV_DIAGNOSTICS_HPP_

#include "high_rate_ev_propagator.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <cstddef>
#include <cstdint>
#include <mutex>

namespace fast_lio
{

struct Phase3PublisherDiagnosticsSnapshot
{
  std::uint64_t last_candidate_sequence{0};
  std::int64_t last_candidate_timestamp_ns{0};
  std::uint64_t last_published_sequence{0};
  std::int64_t last_published_timestamp_ns{0};
  std::size_t worker_queue_depth{0};
  std::size_t worker_queue_max_depth{0};
  double worker_last_processing_s{0.0};
  double worker_max_processing_s{0.0};
  std::uint64_t publish_count{0};
  std::uint64_t reject_count{0};
  std::uint64_t drop_count{0};
  std::uint64_t supersede_count{0};
  std::uint64_t timeout_count{0};
};

class Phase3DiagnosticsStore
{
public:
  void record_worker_processing(double duration_s);
  void record_candidate(
    std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns);
  void record_prepared(
    std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns);
  void record_reject();
  void record_drop();
  void record_dequeued(
    std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns);
  void observe_propagator(
    const HighRateEvPropagator::FixedDiagnosticsSnapshot & snapshot);
  void record_publish(
    std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns);
  Phase3PublisherDiagnosticsSnapshot snapshot() const;

private:
  mutable std::mutex mutex_;
  Phase3PublisherDiagnosticsSnapshot snapshot_;
  std::uint64_t last_timeout_validation_epoch_{0};
};

struct HighRateEvDiagnosticInput
{
  bool phase2_enabled{false};
  bool phase3_enabled{false};
  HighRateEvPropagator::FixedDiagnosticsSnapshot propagator;
  Phase3PublisherDiagnosticsSnapshot publisher;
};

bool high_rate_ev_diagnostics_enabled(bool phase3_enabled);

diagnostic_msgs::msg::DiagnosticArray make_high_rate_ev_diagnostic_array(
  const HighRateEvDiagnosticInput & input,
  const builtin_interfaces::msg::Time & header_stamp);

}  // namespace fast_lio

#endif  // FAST_LIO__HIGH_RATE_EV_DIAGNOSTICS_HPP_

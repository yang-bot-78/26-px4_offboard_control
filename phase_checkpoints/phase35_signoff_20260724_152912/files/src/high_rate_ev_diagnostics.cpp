#include "high_rate_ev_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace fast_lio
{
namespace
{

const char * health_name(
  const HighRateEvPropagator::FixedDiagnosticsSnapshot & snapshot)
{
  using Health = HighRateEvPropagator::Health;
  if (snapshot.health == Health::kDisabled) {return "DISABLED";}
  if (snapshot.health == Health::kInitializing) {return "INITIALIZING";}
  if (snapshot.health == Health::kHealthy) {return "HEALTHY";}
  return snapshot.recovery_started ? "RECOVERING" : "FAULT";
}

const char * fault_name(HighRateEvPropagator::Fault fault)
{
  using Fault = HighRateEvPropagator::Fault;
  switch (fault) {
    case Fault::kNone: return "NONE";
    case Fault::kInvalidInput: return "INVALID_INPUT";
    case Fault::kTimestampOrder: return "TIMESTAMP_ORDER";
    case Fault::kFutureTimestamp: return "FUTURE_TIMESTAMP";
    case Fault::kBufferOverflow: return "BUFFER_OVERFLOW";
    case Fault::kMissingPredecessor: return "MISSING_PREDECESSOR";
    case Fault::kInvalidPosterior: return "INVALID_POSTERIOR";
    case Fault::kImuGap: return "IMU_GAP";
    case Fault::kReplayFailure: return "REPLAY_FAILURE";
    case Fault::kContinuity: return "CONTINUITY";
    case Fault::kLineage: return "LINEAGE";
    case Fault::kImuTimeout: return "IMU_TIMEOUT";
    case Fault::kLidarTimeout: return "LIDAR_TIMEOUT";
    case Fault::kClockJump: return "CLOCK_JUMP";
    case Fault::kEpochMismatch: return "EPOCH_MISMATCH";
    case Fault::kInvalidConfig: return "INVALID_CONFIG";
  }
  return "UNKNOWN";
}

std::string decimal(double value)
{
  if (!std::isfinite(value)) {return "nan";}
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(9) << value;
  return stream.str();
}

template<typename ValueT>
void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const char * key, const ValueT & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  status.values.push_back(std::move(item));
}

void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const char * key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

void add_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const char * key, const char * value)
{
  add_value(status, key, std::string(value));
}

}  // namespace

void Phase3DiagnosticsStore::record_worker_processing(double duration_s)
{
  if (!std::isfinite(duration_s) || duration_s < 0.0) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.worker_last_processing_s = duration_s;
  snapshot_.worker_max_processing_s =
    std::max(snapshot_.worker_max_processing_s, duration_s);
}

void Phase3DiagnosticsStore::record_candidate(
  std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.last_candidate_sequence = candidate_sequence;
  snapshot_.last_candidate_timestamp_ns = candidate_timestamp_ns;
}

void Phase3DiagnosticsStore::record_prepared(
  std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.worker_queue_depth != 0 &&
    (snapshot_.last_candidate_sequence != candidate_sequence ||
    snapshot_.last_candidate_timestamp_ns != candidate_timestamp_ns))
  {
    ++snapshot_.supersede_count;
    ++snapshot_.drop_count;
  }
  snapshot_.last_candidate_sequence = candidate_sequence;
  snapshot_.last_candidate_timestamp_ns = candidate_timestamp_ns;
  snapshot_.worker_queue_depth = 1;
  snapshot_.worker_queue_max_depth =
    std::max(snapshot_.worker_queue_max_depth, snapshot_.worker_queue_depth);
}

void Phase3DiagnosticsStore::record_reject()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++snapshot_.reject_count;
}

void Phase3DiagnosticsStore::record_drop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++snapshot_.drop_count;
}

void Phase3DiagnosticsStore::record_dequeued(
  std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.last_candidate_sequence == candidate_sequence &&
    snapshot_.last_candidate_timestamp_ns == candidate_timestamp_ns)
  {
    snapshot_.worker_queue_depth = 0;
  }
}

void Phase3DiagnosticsStore::observe_propagator(
  const HighRateEvPropagator::FixedDiagnosticsSnapshot & snapshot)
{
  using Fault = HighRateEvPropagator::Fault;
  if (snapshot.fault != Fault::kImuTimeout &&
    snapshot.fault != Fault::kLidarTimeout)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot.validation_epoch != last_timeout_validation_epoch_) {
    ++snapshot_.timeout_count;
    last_timeout_validation_epoch_ = snapshot.validation_epoch;
  }
}

void Phase3DiagnosticsStore::record_publish(
  std::uint64_t candidate_sequence, std::int64_t candidate_timestamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++snapshot_.publish_count;
  snapshot_.last_published_sequence = candidate_sequence;
  snapshot_.last_published_timestamp_ns = candidate_timestamp_ns;
  if (snapshot_.last_candidate_sequence == candidate_sequence &&
    snapshot_.last_candidate_timestamp_ns == candidate_timestamp_ns)
  {
    snapshot_.worker_queue_depth = 0;
  }
}

Phase3PublisherDiagnosticsSnapshot Phase3DiagnosticsStore::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

bool high_rate_ev_diagnostics_enabled(bool phase3_enabled)
{
  return phase3_enabled;
}

diagnostic_msgs::msg::DiagnosticArray make_high_rate_ev_diagnostic_array(
  const HighRateEvDiagnosticInput & input,
  const builtin_interfaces::msg::Time & header_stamp)
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = header_stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "high_rate_ev";
  status.hardware_id = "fast_lio";
  const std::string state = health_name(input.propagator);
  status.message = state;
  if (state == "FAULT") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (state == "HEALTHY") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if (state == "DISABLED") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  status.values.reserve(30);
  add_value(status, "phase2_enabled", input.phase2_enabled ? "true" : "false");
  add_value(status, "phase3_enabled", input.phase3_enabled ? "true" : "false");
  add_value(status, "state", state);
  add_value(status, "fault_reason", std::string(fault_name(input.propagator.fault)));
  add_value(status, "validation_epoch", input.propagator.validation_epoch);
  add_value(status, "generation", input.propagator.generation);
  add_value(status, "parent_generation", input.propagator.parent_generation);
  add_value(
    status, "last_candidate_sequence", input.publisher.last_candidate_sequence);
  add_value(
    status, "last_candidate_timestamp_ns",
    input.publisher.last_candidate_timestamp_ns);
  add_value(
    status, "last_published_sequence", input.publisher.last_published_sequence);
  add_value(
    status, "last_published_timestamp_ns",
    input.publisher.last_published_timestamp_ns);
  add_value(
    status, "correction_age_s", decimal(input.propagator.correction_age_s));
  add_value(status, "imu_age_s", decimal(input.propagator.imu_age_s));
  add_value(status, "lidar_age_s", decimal(input.propagator.lidar_age_s));
  add_value(status, "imu_buffer_count", input.propagator.imu_buffer_count);
  add_value(
    status, "imu_buffer_span_s", decimal(input.propagator.imu_buffer_span_s));
  add_value(status, "pending_watermark", input.propagator.pending_watermark);
  add_value(status, "catch_up_count", input.propagator.catch_up_count);
  add_value(
    status, "worker_queue_depth", input.publisher.worker_queue_depth);
  add_value(
    status, "worker_queue_max_depth", input.publisher.worker_queue_max_depth);
  add_value(
    status, "worker_last_processing_s",
    decimal(input.publisher.worker_last_processing_s));
  add_value(
    status, "worker_max_processing_s",
    decimal(input.publisher.worker_max_processing_s));
  add_value(status, "publish_count", input.publisher.publish_count);
  add_value(status, "reject_count", input.publisher.reject_count);
  add_value(status, "drop_count", input.publisher.drop_count);
  add_value(status, "supersede_count", input.publisher.supersede_count);
  add_value(status, "timeout_count", input.publisher.timeout_count);
  add_value(
    status, "recovery_healthy_s",
    decimal(input.propagator.recovery_healthy_elapsed_s));
  add_value(
    status, "recovery_healthy_required_s",
    decimal(input.propagator.recovery_healthy_required_s));
  add_value(
    status, "recovery_progress", decimal(input.propagator.recovery_progress));
  message.status.push_back(std::move(status));
  return message;
}

}  // namespace fast_lio

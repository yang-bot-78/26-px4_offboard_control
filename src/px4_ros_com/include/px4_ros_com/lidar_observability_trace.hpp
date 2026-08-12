#ifndef PX4_ROS_COM__LIDAR_OBSERVABILITY_TRACE_HPP_
#define PX4_ROS_COM__LIDAR_OBSERVABILITY_TRACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <type_traits>
#include <sys/syscall.h>
#include <unistd.h>

namespace px4_ros_com
{

enum class LidarTraceStage : std::uint8_t {
  kDriverPublishBefore = 1, kDriverPublishAfter = 2,
  kRelayCallbackEntry = 3, kRelayProcessingComplete = 4,
  kRelayPublishBefore = 5, kRelayPublishAfter = 6, kRelayCallbackExit = 7,
  kRawReady = 8, kFreshReady = 9, kRawArrival = 10, kFreshArrival = 11,
  kDriverSignal = 20, kDriverNodeShutdown = 21, kDriverSdkDeinit = 22,
  kDriverDestructor = 23, kDriverProcessExit = 24
};
static_assert(static_cast<std::uint8_t>(LidarTraceStage::kFreshArrival) !=
  static_cast<std::uint8_t>(LidarTraceStage::kDriverSignal), "stage IDs must be unique");

enum class TraceControlState : std::uint8_t {
  kRunning, kFrozen, kFinalDrained, kTerminalPublished,
  kDurableAcked, kTeardownAllowed, kIncomplete
};

// Shared producer-side terminal schema. Authentication and durable storage are
// intentionally performed by the independent capture control plane.
struct TraceTerminalContract {
  std::uint32_t schema_version{1};
  std::string run_id;
  std::string run_nonce;
  std::string stream_id;
  std::uint64_t final_committed_seq{0};
  std::uint64_t terminal_batch_seq{0};
  std::map<std::uint8_t, std::uint64_t> per_stage_committed_counts;
  std::string canonical_records_sha256;
  std::string contract_sha256;
};

inline std::int64_t lidar_trace_steady_now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::int32_t lidar_trace_tid() noexcept {
  return static_cast<std::int32_t>(::syscall(SYS_gettid));
}

struct LidarTraceRecord {
  std::uint64_t local_sequence{0};
  std::int64_t event_ros_ns{-1};
  std::int64_t event_steady_ns{-1};
  std::int64_t header_stamp_ns{-1};
  std::uint64_t timebase{0};
  std::uint64_t occurrence{0};
  std::uint32_t point_num{0};
  std::uint8_t lidar_id{0};
  LidarTraceStage stage{LidarTraceStage::kRawArrival};
  std::int32_t pid{-1};
  std::int32_t tid{-1};
};
static_assert(std::is_trivially_copyable<LidarTraceRecord>::value,
  "LiDAR trace records must stay fixed-layout");

// SPSC, no overwrite: a full ring is observable degradation, never hidden data loss.
template<std::size_t Capacity>
class LidarTraceRing {
public:
  static_assert(Capacity > 1, "ring capacity must be greater than one");
  bool push(LidarTraceRecord record) noexcept {
    attempted_.fetch_add(1, std::memory_order_relaxed);
    if (frozen_.load(std::memory_order_acquire)) {
      rejected_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    writers_.fetch_add(1, std::memory_order_acq_rel);
    if (frozen_.load(std::memory_order_acquire)) {
      writers_.fetch_sub(1, std::memory_order_release);
      rejected_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    const std::size_t write = write_.load(std::memory_order_relaxed);
    const std::size_t next = (write + 1) % Capacity;
    if (next == read_.load(std::memory_order_acquire)) {
      overflow_.fetch_add(1, std::memory_order_relaxed);
      rejected_.fetch_add(1, std::memory_order_relaxed);
      writers_.fetch_sub(1, std::memory_order_release);
      return false;
    }
    record.local_sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    records_[write] = record;
    write_.store(next, std::memory_order_release);
    committed_.fetch_add(1, std::memory_order_relaxed);
    writers_.fetch_sub(1, std::memory_order_release);
    return true;
  }
  bool pop(LidarTraceRecord & record) noexcept {
    const std::size_t read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {return false;}
    record = records_[read];
    read_.store((read + 1) % Capacity, std::memory_order_release);
    return true;
  }
  template<typename Sink>
  bool drain_until(std::uint64_t final_sequence, Sink sink) noexcept {
    LidarTraceRecord record;
    std::uint64_t exported = 0;
    while (pop(record)) {
      if (record.local_sequence > final_sequence || !sink(record)) {return false;}
      exported = record.local_sequence;
    }
    return exported == final_sequence || final_sequence == 0;
  }
  std::uint64_t overflow() const noexcept {return overflow_.load(std::memory_order_relaxed);}
  std::uint64_t attempted() const noexcept {return attempted_.load(std::memory_order_relaxed);}
  std::uint64_t committed() const noexcept {return committed_.load(std::memory_order_relaxed);}
  std::uint64_t rejected() const noexcept {return rejected_.load(std::memory_order_relaxed);}
  // After freeze() succeeds no subsequent producer write may commit. The
  // returned sequence is the final watermark a synchronous drain must export.
  std::uint64_t freeze() noexcept {
    frozen_.store(true, std::memory_order_release);
    while (writers_.load(std::memory_order_acquire) != 0) {}
    return next_sequence_.load(std::memory_order_acquire) - 1;
  }
  bool frozen() const noexcept {return frozen_.load(std::memory_order_acquire);}
private:
  std::array<LidarTraceRecord, Capacity> records_{};
  std::atomic<std::size_t> write_{0}, read_{0};
  std::atomic<std::uint64_t> next_sequence_{1}, overflow_{0}, attempted_{0}, committed_{0}, rejected_{0};
  std::atomic<bool> frozen_{false};
  std::atomic<std::uint32_t> writers_{0};
};

}  // namespace px4_ros_com

#endif

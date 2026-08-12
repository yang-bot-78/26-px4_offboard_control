#ifndef PX4_ROS_COM__FRESH_RELAY_TRACE_HPP_
#define PX4_ROS_COM__FRESH_RELAY_TRACE_HPP_

#include <cstdint>
#include <cstring>
#include <chrono>
#include <limits>
#include <memory>
#include <cstddef>
#include <sys/syscall.h>
#include <unistd.h>
#include <type_traits>
#include <vector>

namespace px4_ros_com
{

enum class RelayImuTraceStage : std::uint8_t
{
  kRmwQueueReady = 1,
  kExecutorDispatch = 2,
  kCallbackEntry = 3,
  kMessageProcessingComplete = 4,
  kPublishCallBefore = 5,
  kPublishReturnAfter = 6,
  kTraceCommitBefore = 7,
  kTraceCommitAfter = 8,
  kCallbackExit = 9,
  kReject = 10
};

inline std::int64_t relay_trace_steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline std::int32_t relay_trace_linux_tid()
{
  return static_cast<std::int32_t>(::syscall(SYS_gettid));
}

struct RelayImuTraceRecord
{
  std::uint64_t relay_trace_sequence{0};
  // This deterministic key is the preserved source timestamp. It is identical
  // in relay and FAST-LIO without altering sensor_msgs/Imu.
  std::uint64_t source_sequence{0};
  std::int64_t source_timestamp_ns{-1};
  std::int64_t event_steady_ns{-1};
  std::int32_t linux_tid{-1};
  RelayImuTraceStage stage{RelayImuTraceStage::kCallbackEntry};
  std::uint8_t reserved[3]{};
};

static_assert(
  std::is_trivially_copyable<RelayImuTraceRecord>::value,
  "relay trace records must remain fixed-layout");

class RelayImuTraceRecorder
{
public:
  explicit RelayImuTraceRecorder(std::size_t capacity)
  : capacity_(capacity), records_(capacity ? new RelayImuTraceRecord[capacity] : nullptr)
  {
  }

  bool enabled() const {return capacity_ != 0;}

  void record(
    RelayImuTraceStage stage, std::int64_t source_timestamp_ns,
    std::int32_t linux_tid,
    std::int64_t steady_ns = relay_trace_steady_now_ns()) noexcept
  {
    if (!enabled()) {return;}
    if (size_ == capacity_) {
      ++dropped_;
      return;
    }
    RelayImuTraceRecord & record = records_[(head_ + size_) % capacity_];
    record.relay_trace_sequence = next_trace_sequence_++;
    record.source_timestamp_ns = source_timestamp_ns;
    record.source_sequence = source_timestamp_ns > 0 ?
      static_cast<std::uint64_t>(source_timestamp_ns) : 0;
    record.event_steady_ns = steady_ns;
    record.linux_tid = linux_tid;
    record.stage = stage;
    ++size_;
  }

  std::size_t take_batch(
    RelayImuTraceRecord * destination, std::size_t maximum) noexcept
  {
    if (!enabled() || destination == nullptr || maximum == 0) {return 0;}
    const std::size_t count = size_ < maximum ? size_ : maximum;
    for (std::size_t index = 0; index < count; ++index) {
      destination[index] = records_[(head_ + index) % capacity_];
    }
    head_ = (head_ + count) % capacity_;
    size_ -= count;
    return count;
  }

  std::uint64_t dropped() const noexcept {return dropped_;}

private:
  std::size_t capacity_{0};
  std::unique_ptr<RelayImuTraceRecord[]> records_;
  std::size_t head_{0};
  std::size_t size_{0};
  std::uint64_t next_trace_sequence_{1};
  std::uint64_t dropped_{0};
};

constexpr std::uint32_t kRelayImuTraceWireMagic = 0x52495452U;  // "RITR"
constexpr std::uint16_t kRelayImuTraceWireVersion = 3;
constexpr std::uint16_t kRelayImuTraceWireHeaderSize = 32;

inline bool pack_relay_imu_trace_batch(
  std::vector<std::uint8_t> & destination, const RelayImuTraceRecord * records,
  std::size_t count, std::uint64_t batch_id, std::uint64_t dropped)
{
  if (records == nullptr || count > std::numeric_limits<std::uint32_t>::max()) {
    return false;
  }
  const std::size_t required = kRelayImuTraceWireHeaderSize +
    count * sizeof(RelayImuTraceRecord);
  if (destination.capacity() < required) {return false;}
  destination.resize(required);
  const std::uint32_t wire_count = static_cast<std::uint32_t>(count);
  const std::uint32_t record_size = sizeof(RelayImuTraceRecord);
  std::size_t offset = 0;
  const auto write = [&destination, &offset](const auto & value) {
      std::memcpy(destination.data() + offset, &value, sizeof(value));
      offset += sizeof(value);
    };
  write(kRelayImuTraceWireMagic);
  write(kRelayImuTraceWireVersion);
  write(kRelayImuTraceWireHeaderSize);
  write(record_size);
  write(wire_count);
  write(batch_id);
  write(dropped);
  for (std::size_t index = 0; index < count; ++index) {
    std::memcpy(destination.data() + offset, &records[index], sizeof(records[index]));
    offset += sizeof(records[index]);
  }
  return true;
}

}  // namespace px4_ros_com

#endif  // PX4_ROS_COM__FRESH_RELAY_TRACE_HPP_

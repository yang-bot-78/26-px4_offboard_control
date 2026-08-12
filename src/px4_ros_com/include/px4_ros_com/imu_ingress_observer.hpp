#ifndef PX4_ROS_COM__IMU_INGRESS_OBSERVER_HPP_
#define PX4_ROS_COM__IMU_INGRESS_OBSERVER_HPP_

#include "px4_ros_com/fresh_relay_trace.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace px4_ros_com
{

class ImuIngressObserver
{
public:
  static constexpr std::size_t kReadyTokenCapacity = 2048;

  ImuIngressObserver()
  {
    for (std::size_t index = 0; index < kReadyTokenCapacity; ++index) {
      tokens_[index].sequence.store(index, std::memory_order_relaxed);
    }
  }

  struct Observation
  {
    std::int64_t ready_steady_ns{-1};
    std::int64_t dispatch_steady_ns{-1};
    bool ready_correlated{false};
  };

  void set_target(const rclcpp::SubscriptionBase * subscription) noexcept
  {
    target_ = subscription;
  }

  void on_ready(std::size_t count) noexcept
  {
    const std::int64_t now = relay_trace_steady_now_ns();
    for (std::size_t index = 0; index < count; ++index) {
      ++ready_notifications_;
      if (!enqueue(now)) {++ready_overflow_;}
    }
  }

  void before_dispatch(const rclcpp::SubscriptionBase::SharedPtr & subscription) noexcept
  {
    if (!subscription || subscription.get() != target_) {return;}
    if (pending_valid_) {++dispatch_unclaimed_;}
    pending_ = Observation{};
    pending_.dispatch_steady_ns = relay_trace_steady_now_ns();
    pending_.ready_correlated = dequeue(&pending_.ready_steady_ns);
    if (!pending_.ready_correlated) {++dispatch_without_ready_;}
    pending_valid_ = true;
    ++dispatches_;
  }

  Observation claim() noexcept
  {
    if (!pending_valid_) {
      ++callback_without_dispatch_;
      return Observation{};
    }
    pending_valid_ = false;
    if (pending_.ready_correlated) {++correlated_callbacks_;}
    return pending_;
  }

  struct Counters
  {
    std::uint64_t ready_notifications{0};
    std::uint64_t ready_overflow{0};
    std::uint64_t dispatches{0};
    std::uint64_t dispatch_without_ready{0};
    std::uint64_t dispatch_unclaimed{0};
    std::uint64_t callback_without_dispatch{0};
    std::uint64_t correlated_callbacks{0};
  };

  Counters counters() const noexcept
  {
    return {ready_notifications_.load(), ready_overflow_.load(), dispatches_.load(),
      dispatch_without_ready_.load(), dispatch_unclaimed_.load(),
      callback_without_dispatch_.load(), correlated_callbacks_.load()};
  }

private:
  struct Token
  {
    std::atomic<std::size_t> sequence{0};
    std::int64_t steady_ns{-1};
  };

  bool enqueue(std::int64_t steady_ns) noexcept
  {
    std::size_t position = write_.load(std::memory_order_relaxed);
    for (;;) {
      Token & token = tokens_[position & (kReadyTokenCapacity - 1)];
      const std::intptr_t difference = static_cast<std::intptr_t>(
        token.sequence.load(std::memory_order_acquire)) - static_cast<std::intptr_t>(position);
      if (difference == 0) {
        if (write_.compare_exchange_weak(
            position, position + 1, std::memory_order_relaxed, std::memory_order_relaxed)) {
          token.steady_ns = steady_ns;
          token.sequence.store(position + 1, std::memory_order_release);
          return true;
        }
      } else if (difference < 0) {
        return false;
      } else {
        position = write_.load(std::memory_order_relaxed);
      }
    }
  }

  bool dequeue(std::int64_t * steady_ns) noexcept
  {
    std::size_t position = read_.load(std::memory_order_relaxed);
    for (;;) {
      Token & token = tokens_[position & (kReadyTokenCapacity - 1)];
      const std::intptr_t difference = static_cast<std::intptr_t>(
        token.sequence.load(std::memory_order_acquire)) -
        static_cast<std::intptr_t>(position + 1);
      if (difference == 0) {
        if (read_.compare_exchange_weak(
            position, position + 1, std::memory_order_relaxed, std::memory_order_relaxed)) {
          *steady_ns = token.steady_ns;
          token.sequence.store(position + kReadyTokenCapacity, std::memory_order_release);
          return true;
        }
      } else if (difference < 0) {
        return false;
      } else {
        position = read_.load(std::memory_order_relaxed);
      }
    }
  }

  const rclcpp::SubscriptionBase * target_{nullptr};
  std::array<Token, kReadyTokenCapacity> tokens_{};
  std::atomic<std::size_t> write_{0};
  std::atomic<std::size_t> read_{0};
  std::atomic<std::uint64_t> ready_notifications_{0};
  std::atomic<std::uint64_t> ready_overflow_{0};
  std::atomic<std::uint64_t> dispatches_{0};
  std::atomic<std::uint64_t> dispatch_without_ready_{0};
  std::atomic<std::uint64_t> dispatch_unclaimed_{0};
  std::atomic<std::uint64_t> callback_without_dispatch_{0};
  std::atomic<std::uint64_t> correlated_callbacks_{0};
  Observation pending_{};
  bool pending_valid_{false};
};

class ObservedSingleThreadedExecutor final : public rclcpp::executors::SingleThreadedExecutor
{
public:
  explicit ObservedSingleThreadedExecutor(ImuIngressObserver * observer)
  : observer_(observer) {}

  void spin() override
  {
    if (spinning.exchange(true)) {throw std::runtime_error("spin already active");}
    while (rclcpp::ok(context_) && spinning.load()) {
      rclcpp::AnyExecutable executable;
      if (get_next_executable(executable)) {
        if (observer_ && executable.subscription) {observer_->before_dispatch(executable.subscription);}
        execute_any_executable(executable);
      }
    }
    spinning.store(false);
  }

private:
  ImuIngressObserver * observer_{nullptr};
};

}  // namespace px4_ros_com

#endif  // PX4_ROS_COM__IMU_INGRESS_OBSERVER_HPP_

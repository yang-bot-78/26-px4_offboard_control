// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from race_msgs:msg/GlobalPlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__STRUCT_HPP_
#define RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__race_msgs__msg__GlobalPlannerStatus __attribute__((deprecated))
#else
# define DEPRECATED__race_msgs__msg__GlobalPlannerStatus __declspec(deprecated)
#endif

namespace race_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GlobalPlannerStatus_
{
  using Type = GlobalPlannerStatus_<ContainerAllocator>;

  explicit GlobalPlannerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->global_goal_id = 0ull;
      this->global_path_id = 0ull;
      this->local_goal_seq = 0ull;
      this->goal_active = false;
      this->final_goal_reached = false;
      this->distance_to_final = 0.0;
      this->horizontal_speed = 0.0;
      this->mode = "";
      this->reason = "";
    }
  }

  explicit GlobalPlannerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    mode(_alloc),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->global_goal_id = 0ull;
      this->global_path_id = 0ull;
      this->local_goal_seq = 0ull;
      this->goal_active = false;
      this->final_goal_reached = false;
      this->distance_to_final = 0.0;
      this->horizontal_speed = 0.0;
      this->mode = "";
      this->reason = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _global_goal_id_type =
    uint64_t;
  _global_goal_id_type global_goal_id;
  using _global_path_id_type =
    uint64_t;
  _global_path_id_type global_path_id;
  using _local_goal_seq_type =
    uint64_t;
  _local_goal_seq_type local_goal_seq;
  using _goal_active_type =
    bool;
  _goal_active_type goal_active;
  using _final_goal_reached_type =
    bool;
  _final_goal_reached_type final_goal_reached;
  using _distance_to_final_type =
    double;
  _distance_to_final_type distance_to_final;
  using _horizontal_speed_type =
    double;
  _horizontal_speed_type horizontal_speed;
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__global_goal_id(
    const uint64_t & _arg)
  {
    this->global_goal_id = _arg;
    return *this;
  }
  Type & set__global_path_id(
    const uint64_t & _arg)
  {
    this->global_path_id = _arg;
    return *this;
  }
  Type & set__local_goal_seq(
    const uint64_t & _arg)
  {
    this->local_goal_seq = _arg;
    return *this;
  }
  Type & set__goal_active(
    const bool & _arg)
  {
    this->goal_active = _arg;
    return *this;
  }
  Type & set__final_goal_reached(
    const bool & _arg)
  {
    this->final_goal_reached = _arg;
    return *this;
  }
  Type & set__distance_to_final(
    const double & _arg)
  {
    this->distance_to_final = _arg;
    return *this;
  }
  Type & set__horizontal_speed(
    const double & _arg)
  {
    this->horizontal_speed = _arg;
    return *this;
  }
  Type & set__mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__race_msgs__msg__GlobalPlannerStatus
    std::shared_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__race_msgs__msg__GlobalPlannerStatus
    std::shared_ptr<race_msgs::msg::GlobalPlannerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GlobalPlannerStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->global_goal_id != other.global_goal_id) {
      return false;
    }
    if (this->global_path_id != other.global_path_id) {
      return false;
    }
    if (this->local_goal_seq != other.local_goal_seq) {
      return false;
    }
    if (this->goal_active != other.goal_active) {
      return false;
    }
    if (this->final_goal_reached != other.final_goal_reached) {
      return false;
    }
    if (this->distance_to_final != other.distance_to_final) {
      return false;
    }
    if (this->horizontal_speed != other.horizontal_speed) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const GlobalPlannerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GlobalPlannerStatus_

// alias to use template instance with default allocator
using GlobalPlannerStatus =
  race_msgs::msg::GlobalPlannerStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__STRUCT_HPP_

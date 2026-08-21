// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/StatusData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__STRUCT_HPP_

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
# define DEPRECATED__quadrotor_msgs__msg__StatusData __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__StatusData __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StatusData_
{
  using Type = StatusData_<ContainerAllocator>;

  explicit StatusData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loop_rate = 0;
      this->voltage = 0.0;
      this->seq = 0;
    }
  }

  explicit StatusData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loop_rate = 0;
      this->voltage = 0.0;
      this->seq = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _loop_rate_type =
    uint16_t;
  _loop_rate_type loop_rate;
  using _voltage_type =
    double;
  _voltage_type voltage;
  using _seq_type =
    uint8_t;
  _seq_type seq;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__loop_rate(
    const uint16_t & _arg)
  {
    this->loop_rate = _arg;
    return *this;
  }
  Type & set__voltage(
    const double & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__seq(
    const uint8_t & _arg)
  {
    this->seq = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::StatusData_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::StatusData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::StatusData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::StatusData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__StatusData
    std::shared_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__StatusData
    std::shared_ptr<quadrotor_msgs::msg::StatusData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StatusData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->loop_rate != other.loop_rate) {
      return false;
    }
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->seq != other.seq) {
      return false;
    }
    return true;
  }
  bool operator!=(const StatusData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StatusData_

// alias to use template instance with default allocator
using StatusData =
  quadrotor_msgs::msg::StatusData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/Gains.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__GAINS__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__GAINS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__Gains __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__Gains __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gains_
{
  using Type = Gains_<ContainerAllocator>;

  explicit Gains_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->kp = 0.0;
      this->kd = 0.0;
      this->kp_yaw = 0.0;
      this->kd_yaw = 0.0;
    }
  }

  explicit Gains_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->kp = 0.0;
      this->kd = 0.0;
      this->kp_yaw = 0.0;
      this->kd_yaw = 0.0;
    }
  }

  // field types and members
  using _kp_type =
    double;
  _kp_type kp;
  using _kd_type =
    double;
  _kd_type kd;
  using _kp_yaw_type =
    double;
  _kp_yaw_type kp_yaw;
  using _kd_yaw_type =
    double;
  _kd_yaw_type kd_yaw;

  // setters for named parameter idiom
  Type & set__kp(
    const double & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const double & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__kp_yaw(
    const double & _arg)
  {
    this->kp_yaw = _arg;
    return *this;
  }
  Type & set__kd_yaw(
    const double & _arg)
  {
    this->kd_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::Gains_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::Gains_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::Gains_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::Gains_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__Gains
    std::shared_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__Gains
    std::shared_ptr<quadrotor_msgs::msg::Gains_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gains_ & other) const
  {
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->kp_yaw != other.kp_yaw) {
      return false;
    }
    if (this->kd_yaw != other.kd_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gains_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gains_

// alias to use template instance with default allocator
using Gains =
  quadrotor_msgs::msg::Gains_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__GAINS__STRUCT_HPP_

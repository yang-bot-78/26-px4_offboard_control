// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from race_msgs:msg/NavigationSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__STRUCT_HPP_
#define RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__race_msgs__msg__NavigationSetpoint __attribute__((deprecated))
#else
# define DEPRECATED__race_msgs__msg__NavigationSetpoint __declspec(deprecated)
#endif

namespace race_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavigationSetpoint_
{
  using Type = NavigationSetpoint_<ContainerAllocator>;

  explicit NavigationSetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity_valid = false;
      this->yaw = 0.0;
      this->yaw_rate_valid = false;
      this->yaw_rate = 0.0;
    }
  }

  explicit NavigationSetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity_valid = false;
      this->yaw = 0.0;
      this->yaw_rate_valid = false;
      this->yaw_rate = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_valid_type =
    bool;
  _velocity_valid_type velocity_valid;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _yaw_rate_valid_type =
    bool;
  _yaw_rate_valid_type yaw_rate_valid;
  using _yaw_rate_type =
    double;
  _yaw_rate_type yaw_rate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity_valid(
    const bool & _arg)
  {
    this->velocity_valid = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__yaw_rate_valid(
    const bool & _arg)
  {
    this->yaw_rate_valid = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const double & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    race_msgs::msg::NavigationSetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const race_msgs::msg::NavigationSetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::NavigationSetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::NavigationSetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__race_msgs__msg__NavigationSetpoint
    std::shared_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__race_msgs__msg__NavigationSetpoint
    std::shared_ptr<race_msgs::msg::NavigationSetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigationSetpoint_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity_valid != other.velocity_valid) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->yaw_rate_valid != other.yaw_rate_valid) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigationSetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigationSetpoint_

// alias to use template instance with default allocator
using NavigationSetpoint =
  race_msgs::msg::NavigationSetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__STRUCT_HPP_

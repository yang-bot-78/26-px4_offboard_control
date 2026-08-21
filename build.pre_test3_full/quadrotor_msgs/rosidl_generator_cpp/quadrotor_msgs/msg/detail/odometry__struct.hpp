// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'curodom'
// Member 'kfodom'
#include "nav_msgs/msg/detail/odometry__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__Odometry __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__Odometry __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Odometry_
{
  using Type = Odometry_<ContainerAllocator>;

  explicit Odometry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : curodom(_init),
    kfodom(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->kfid = 0ul;
      this->status = 0;
    }
  }

  explicit Odometry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : curodom(_alloc, _init),
    kfodom(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->kfid = 0ul;
      this->status = 0;
    }
  }

  // field types and members
  using _curodom_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _curodom_type curodom;
  using _kfodom_type =
    nav_msgs::msg::Odometry_<ContainerAllocator>;
  _kfodom_type kfodom;
  using _kfid_type =
    uint32_t;
  _kfid_type kfid;
  using _status_type =
    uint8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__curodom(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->curodom = _arg;
    return *this;
  }
  Type & set__kfodom(
    const nav_msgs::msg::Odometry_<ContainerAllocator> & _arg)
  {
    this->kfodom = _arg;
    return *this;
  }
  Type & set__kfid(
    const uint32_t & _arg)
  {
    this->kfid = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATUS_ODOM_VALID =
    0u;
  static constexpr uint8_t STATUS_ODOM_INVALID =
    1u;
  static constexpr uint8_t STATUS_ODOM_LOOPCLOSURE =
    2u;

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::Odometry_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::Odometry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::Odometry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::Odometry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__Odometry
    std::shared_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__Odometry
    std::shared_ptr<quadrotor_msgs::msg::Odometry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Odometry_ & other) const
  {
    if (this->curodom != other.curodom) {
      return false;
    }
    if (this->kfodom != other.kfodom) {
      return false;
    }
    if (this->kfid != other.kfid) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Odometry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Odometry_

// alias to use template instance with default allocator
using Odometry =
  quadrotor_msgs::msg::Odometry_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Odometry_<ContainerAllocator>::STATUS_ODOM_VALID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Odometry_<ContainerAllocator>::STATUS_ODOM_INVALID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Odometry_<ContainerAllocator>::STATUS_ODOM_LOOPCLOSURE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_HPP_

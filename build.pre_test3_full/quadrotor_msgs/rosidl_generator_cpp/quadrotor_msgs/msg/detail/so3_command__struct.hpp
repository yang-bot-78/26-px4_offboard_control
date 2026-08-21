// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/SO3Command.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__STRUCT_HPP_

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
// Member 'force'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'aux'
#include "quadrotor_msgs/msg/detail/aux_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__SO3Command __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__SO3Command __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SO3Command_
{
  using Type = SO3Command_<ContainerAllocator>;

  explicit SO3Command_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    force(_init),
    orientation(_init),
    aux(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->kr.begin(), this->kr.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->kom.begin(), this->kom.end(), 0.0);
    }
  }

  explicit SO3Command_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    force(_alloc, _init),
    orientation(_alloc, _init),
    kr(_alloc),
    kom(_alloc),
    aux(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->kr.begin(), this->kr.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->kom.begin(), this->kom.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _force_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _force_type force;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _kr_type =
    std::array<double, 3>;
  _kr_type kr;
  using _kom_type =
    std::array<double, 3>;
  _kom_type kom;
  using _aux_type =
    quadrotor_msgs::msg::AuxCommand_<ContainerAllocator>;
  _aux_type aux;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__force(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->force = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__kr(
    const std::array<double, 3> & _arg)
  {
    this->kr = _arg;
    return *this;
  }
  Type & set__kom(
    const std::array<double, 3> & _arg)
  {
    this->kom = _arg;
    return *this;
  }
  Type & set__aux(
    const quadrotor_msgs::msg::AuxCommand_<ContainerAllocator> & _arg)
  {
    this->aux = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::SO3Command_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::SO3Command_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::SO3Command_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::SO3Command_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__SO3Command
    std::shared_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__SO3Command
    std::shared_ptr<quadrotor_msgs::msg::SO3Command_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SO3Command_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->force != other.force) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->kr != other.kr) {
      return false;
    }
    if (this->kom != other.kom) {
      return false;
    }
    if (this->aux != other.aux) {
      return false;
    }
    return true;
  }
  bool operator!=(const SO3Command_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SO3Command_

// alias to use template instance with default allocator
using SO3Command =
  quadrotor_msgs::msg::SO3Command_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__STRUCT_HPP_

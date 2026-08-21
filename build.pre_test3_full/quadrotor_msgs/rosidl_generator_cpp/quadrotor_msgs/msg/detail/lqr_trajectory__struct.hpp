// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/LQRTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__STRUCT_HPP_

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
# define DEPRECATED__quadrotor_msgs__msg__LQRTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__LQRTrajectory __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LQRTrajectory_
{
  using Type = LQRTrajectory_<ContainerAllocator>;

  explicit LQRTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0ul;
      this->action = 0ul;
      this->r = 0.0;
      this->start_yaw = 0.0;
      this->final_yaw = 0.0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->s0.begin(), this->s0.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->ut.begin(), this->ut.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->sf.begin(), this->sf.end(), 0.0);
      this->t_f = 0.0;
      this->debug_info = "";
    }
  }

  explicit LQRTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    s0(_alloc),
    ut(_alloc),
    sf(_alloc),
    debug_info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0ul;
      this->action = 0ul;
      this->r = 0.0;
      this->start_yaw = 0.0;
      this->final_yaw = 0.0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->s0.begin(), this->s0.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->ut.begin(), this->ut.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->sf.begin(), this->sf.end(), 0.0);
      this->t_f = 0.0;
      this->debug_info = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trajectory_id_type =
    uint32_t;
  _trajectory_id_type trajectory_id;
  using _action_type =
    uint32_t;
  _action_type action;
  using _r_type =
    double;
  _r_type r;
  using _start_yaw_type =
    double;
  _start_yaw_type start_yaw;
  using _final_yaw_type =
    double;
  _final_yaw_type final_yaw;
  using _s0_type =
    std::array<double, 6>;
  _s0_type s0;
  using _ut_type =
    std::array<double, 3>;
  _ut_type ut;
  using _sf_type =
    std::array<double, 6>;
  _sf_type sf;
  using _t_f_type =
    double;
  _t_f_type t_f;
  using _debug_info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _debug_info_type debug_info;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__trajectory_id(
    const uint32_t & _arg)
  {
    this->trajectory_id = _arg;
    return *this;
  }
  Type & set__action(
    const uint32_t & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__r(
    const double & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__start_yaw(
    const double & _arg)
  {
    this->start_yaw = _arg;
    return *this;
  }
  Type & set__final_yaw(
    const double & _arg)
  {
    this->final_yaw = _arg;
    return *this;
  }
  Type & set__s0(
    const std::array<double, 6> & _arg)
  {
    this->s0 = _arg;
    return *this;
  }
  Type & set__ut(
    const std::array<double, 3> & _arg)
  {
    this->ut = _arg;
    return *this;
  }
  Type & set__sf(
    const std::array<double, 6> & _arg)
  {
    this->sf = _arg;
    return *this;
  }
  Type & set__t_f(
    const double & _arg)
  {
    this->t_f = _arg;
    return *this;
  }
  Type & set__debug_info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->debug_info = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t ACTION_ADD =
    1u;
  static constexpr uint32_t ACTION_ABORT =
    2u;
  static constexpr uint32_t ACTION_WARN_START =
    3u;
  static constexpr uint32_t ACTION_WARN_FINAL =
    4u;
  static constexpr uint32_t ACTION_WARN_IMPOSSIBLE =
    5u;

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__LQRTrajectory
    std::shared_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__LQRTrajectory
    std::shared_ptr<quadrotor_msgs::msg::LQRTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LQRTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trajectory_id != other.trajectory_id) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->start_yaw != other.start_yaw) {
      return false;
    }
    if (this->final_yaw != other.final_yaw) {
      return false;
    }
    if (this->s0 != other.s0) {
      return false;
    }
    if (this->ut != other.ut) {
      return false;
    }
    if (this->sf != other.sf) {
      return false;
    }
    if (this->t_f != other.t_f) {
      return false;
    }
    if (this->debug_info != other.debug_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const LQRTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LQRTrajectory_

// alias to use template instance with default allocator
using LQRTrajectory =
  quadrotor_msgs::msg::LQRTrajectory_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t LQRTrajectory_<ContainerAllocator>::ACTION_ADD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t LQRTrajectory_<ContainerAllocator>::ACTION_ABORT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t LQRTrajectory_<ContainerAllocator>::ACTION_WARN_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t LQRTrajectory_<ContainerAllocator>::ACTION_WARN_FINAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t LQRTrajectory_<ContainerAllocator>::ACTION_WARN_IMPOSSIBLE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_HPP_

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
# define DEPRECATED__quadrotor_msgs__msg__PolynomialTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__PolynomialTrajectory __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolynomialTrajectory_
{
  using Type = PolynomialTrajectory_<ContainerAllocator>;

  explicit PolynomialTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0ul;
      this->action = 0ul;
      this->num_order = 0ul;
      this->num_segment = 0ul;
      this->start_yaw = 0.0;
      this->final_yaw = 0.0;
      this->mag_coeff = 0.0;
      this->debug_info = "";
    }
  }

  explicit PolynomialTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    debug_info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0ul;
      this->action = 0ul;
      this->num_order = 0ul;
      this->num_segment = 0ul;
      this->start_yaw = 0.0;
      this->final_yaw = 0.0;
      this->mag_coeff = 0.0;
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
  using _num_order_type =
    uint32_t;
  _num_order_type num_order;
  using _num_segment_type =
    uint32_t;
  _num_segment_type num_segment;
  using _start_yaw_type =
    double;
  _start_yaw_type start_yaw;
  using _final_yaw_type =
    double;
  _final_yaw_type final_yaw;
  using _coef_x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _coef_x_type coef_x;
  using _coef_y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _coef_y_type coef_y;
  using _coef_z_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _coef_z_type coef_z;
  using _time_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _time_type time;
  using _mag_coeff_type =
    double;
  _mag_coeff_type mag_coeff;
  using _order_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _order_type order;
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
  Type & set__num_order(
    const uint32_t & _arg)
  {
    this->num_order = _arg;
    return *this;
  }
  Type & set__num_segment(
    const uint32_t & _arg)
  {
    this->num_segment = _arg;
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
  Type & set__coef_x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->coef_x = _arg;
    return *this;
  }
  Type & set__coef_y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->coef_y = _arg;
    return *this;
  }
  Type & set__coef_z(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->coef_z = _arg;
    return *this;
  }
  Type & set__time(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__mag_coeff(
    const double & _arg)
  {
    this->mag_coeff = _arg;
    return *this;
  }
  Type & set__order(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->order = _arg;
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
    quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__PolynomialTrajectory
    std::shared_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__PolynomialTrajectory
    std::shared_ptr<quadrotor_msgs::msg::PolynomialTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolynomialTrajectory_ & other) const
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
    if (this->num_order != other.num_order) {
      return false;
    }
    if (this->num_segment != other.num_segment) {
      return false;
    }
    if (this->start_yaw != other.start_yaw) {
      return false;
    }
    if (this->final_yaw != other.final_yaw) {
      return false;
    }
    if (this->coef_x != other.coef_x) {
      return false;
    }
    if (this->coef_y != other.coef_y) {
      return false;
    }
    if (this->coef_z != other.coef_z) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    if (this->mag_coeff != other.mag_coeff) {
      return false;
    }
    if (this->order != other.order) {
      return false;
    }
    if (this->debug_info != other.debug_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolynomialTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolynomialTrajectory_

// alias to use template instance with default allocator
using PolynomialTrajectory =
  quadrotor_msgs::msg::PolynomialTrajectory_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t PolynomialTrajectory_<ContainerAllocator>::ACTION_ADD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t PolynomialTrajectory_<ContainerAllocator>::ACTION_ABORT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t PolynomialTrajectory_<ContainerAllocator>::ACTION_WARN_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t PolynomialTrajectory_<ContainerAllocator>::ACTION_WARN_FINAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t PolynomialTrajectory_<ContainerAllocator>::ACTION_WARN_IMPOSSIBLE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_HPP_

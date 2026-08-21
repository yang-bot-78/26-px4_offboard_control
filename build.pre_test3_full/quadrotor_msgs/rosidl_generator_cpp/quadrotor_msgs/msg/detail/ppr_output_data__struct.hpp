// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/PPROutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__STRUCT_HPP_

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
# define DEPRECATED__quadrotor_msgs__msg__PPROutputData __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__PPROutputData __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PPROutputData_
{
  using Type = PPROutputData_<ContainerAllocator>;

  explicit PPROutputData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quad_time = 0;
      this->des_thrust = 0.0;
      this->des_roll = 0.0;
      this->des_pitch = 0.0;
      this->des_yaw = 0.0;
      this->est_roll = 0.0;
      this->est_pitch = 0.0;
      this->est_yaw = 0.0;
      this->est_angvel_x = 0.0;
      this->est_angvel_y = 0.0;
      this->est_angvel_z = 0.0;
      this->est_acc_x = 0.0;
      this->est_acc_y = 0.0;
      this->est_acc_z = 0.0;
      std::fill<typename std::array<uint16_t, 4>::iterator, uint16_t>(this->pwm.begin(), this->pwm.end(), 0);
    }
  }

  explicit PPROutputData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pwm(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quad_time = 0;
      this->des_thrust = 0.0;
      this->des_roll = 0.0;
      this->des_pitch = 0.0;
      this->des_yaw = 0.0;
      this->est_roll = 0.0;
      this->est_pitch = 0.0;
      this->est_yaw = 0.0;
      this->est_angvel_x = 0.0;
      this->est_angvel_y = 0.0;
      this->est_angvel_z = 0.0;
      this->est_acc_x = 0.0;
      this->est_acc_y = 0.0;
      this->est_acc_z = 0.0;
      std::fill<typename std::array<uint16_t, 4>::iterator, uint16_t>(this->pwm.begin(), this->pwm.end(), 0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _quad_time_type =
    uint16_t;
  _quad_time_type quad_time;
  using _des_thrust_type =
    double;
  _des_thrust_type des_thrust;
  using _des_roll_type =
    double;
  _des_roll_type des_roll;
  using _des_pitch_type =
    double;
  _des_pitch_type des_pitch;
  using _des_yaw_type =
    double;
  _des_yaw_type des_yaw;
  using _est_roll_type =
    double;
  _est_roll_type est_roll;
  using _est_pitch_type =
    double;
  _est_pitch_type est_pitch;
  using _est_yaw_type =
    double;
  _est_yaw_type est_yaw;
  using _est_angvel_x_type =
    double;
  _est_angvel_x_type est_angvel_x;
  using _est_angvel_y_type =
    double;
  _est_angvel_y_type est_angvel_y;
  using _est_angvel_z_type =
    double;
  _est_angvel_z_type est_angvel_z;
  using _est_acc_x_type =
    double;
  _est_acc_x_type est_acc_x;
  using _est_acc_y_type =
    double;
  _est_acc_y_type est_acc_y;
  using _est_acc_z_type =
    double;
  _est_acc_z_type est_acc_z;
  using _pwm_type =
    std::array<uint16_t, 4>;
  _pwm_type pwm;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__quad_time(
    const uint16_t & _arg)
  {
    this->quad_time = _arg;
    return *this;
  }
  Type & set__des_thrust(
    const double & _arg)
  {
    this->des_thrust = _arg;
    return *this;
  }
  Type & set__des_roll(
    const double & _arg)
  {
    this->des_roll = _arg;
    return *this;
  }
  Type & set__des_pitch(
    const double & _arg)
  {
    this->des_pitch = _arg;
    return *this;
  }
  Type & set__des_yaw(
    const double & _arg)
  {
    this->des_yaw = _arg;
    return *this;
  }
  Type & set__est_roll(
    const double & _arg)
  {
    this->est_roll = _arg;
    return *this;
  }
  Type & set__est_pitch(
    const double & _arg)
  {
    this->est_pitch = _arg;
    return *this;
  }
  Type & set__est_yaw(
    const double & _arg)
  {
    this->est_yaw = _arg;
    return *this;
  }
  Type & set__est_angvel_x(
    const double & _arg)
  {
    this->est_angvel_x = _arg;
    return *this;
  }
  Type & set__est_angvel_y(
    const double & _arg)
  {
    this->est_angvel_y = _arg;
    return *this;
  }
  Type & set__est_angvel_z(
    const double & _arg)
  {
    this->est_angvel_z = _arg;
    return *this;
  }
  Type & set__est_acc_x(
    const double & _arg)
  {
    this->est_acc_x = _arg;
    return *this;
  }
  Type & set__est_acc_y(
    const double & _arg)
  {
    this->est_acc_y = _arg;
    return *this;
  }
  Type & set__est_acc_z(
    const double & _arg)
  {
    this->est_acc_z = _arg;
    return *this;
  }
  Type & set__pwm(
    const std::array<uint16_t, 4> & _arg)
  {
    this->pwm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::PPROutputData_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::PPROutputData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::PPROutputData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::PPROutputData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__PPROutputData
    std::shared_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__PPROutputData
    std::shared_ptr<quadrotor_msgs::msg::PPROutputData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PPROutputData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->quad_time != other.quad_time) {
      return false;
    }
    if (this->des_thrust != other.des_thrust) {
      return false;
    }
    if (this->des_roll != other.des_roll) {
      return false;
    }
    if (this->des_pitch != other.des_pitch) {
      return false;
    }
    if (this->des_yaw != other.des_yaw) {
      return false;
    }
    if (this->est_roll != other.est_roll) {
      return false;
    }
    if (this->est_pitch != other.est_pitch) {
      return false;
    }
    if (this->est_yaw != other.est_yaw) {
      return false;
    }
    if (this->est_angvel_x != other.est_angvel_x) {
      return false;
    }
    if (this->est_angvel_y != other.est_angvel_y) {
      return false;
    }
    if (this->est_angvel_z != other.est_angvel_z) {
      return false;
    }
    if (this->est_acc_x != other.est_acc_x) {
      return false;
    }
    if (this->est_acc_y != other.est_acc_y) {
      return false;
    }
    if (this->est_acc_z != other.est_acc_z) {
      return false;
    }
    if (this->pwm != other.pwm) {
      return false;
    }
    return true;
  }
  bool operator!=(const PPROutputData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PPROutputData_

// alias to use template instance with default allocator
using PPROutputData =
  quadrotor_msgs::msg::PPROutputData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__STRUCT_HPP_

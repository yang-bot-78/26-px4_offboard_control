// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/OutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__STRUCT_HPP_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'angular_velocity'
// Member 'linear_acceleration'
// Member 'magnetic_field'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__OutputData __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__OutputData __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OutputData_
{
  using Type = OutputData_<ContainerAllocator>;

  explicit OutputData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    orientation(_init),
    angular_velocity(_init),
    linear_acceleration(_init),
    magnetic_field(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loop_rate = 0;
      this->voltage = 0.0;
      this->pressure_dheight = 0.0;
      this->pressure_height = 0.0;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->radio_channel.begin(), this->radio_channel.end(), 0);
      this->seq = 0;
    }
  }

  explicit OutputData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    orientation(_alloc, _init),
    angular_velocity(_alloc, _init),
    linear_acceleration(_alloc, _init),
    magnetic_field(_alloc, _init),
    radio_channel(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loop_rate = 0;
      this->voltage = 0.0;
      this->pressure_dheight = 0.0;
      this->pressure_height = 0.0;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->radio_channel.begin(), this->radio_channel.end(), 0);
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
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;
  using _linear_acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_acceleration_type linear_acceleration;
  using _pressure_dheight_type =
    double;
  _pressure_dheight_type pressure_dheight;
  using _pressure_height_type =
    double;
  _pressure_height_type pressure_height;
  using _magnetic_field_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _magnetic_field_type magnetic_field;
  using _radio_channel_type =
    std::array<uint8_t, 8>;
  _radio_channel_type radio_channel;
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
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__linear_acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear_acceleration = _arg;
    return *this;
  }
  Type & set__pressure_dheight(
    const double & _arg)
  {
    this->pressure_dheight = _arg;
    return *this;
  }
  Type & set__pressure_height(
    const double & _arg)
  {
    this->pressure_height = _arg;
    return *this;
  }
  Type & set__magnetic_field(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->magnetic_field = _arg;
    return *this;
  }
  Type & set__radio_channel(
    const std::array<uint8_t, 8> & _arg)
  {
    this->radio_channel = _arg;
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
    quadrotor_msgs::msg::OutputData_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::OutputData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::OutputData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::OutputData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__OutputData
    std::shared_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__OutputData
    std::shared_ptr<quadrotor_msgs::msg::OutputData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OutputData_ & other) const
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
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->linear_acceleration != other.linear_acceleration) {
      return false;
    }
    if (this->pressure_dheight != other.pressure_dheight) {
      return false;
    }
    if (this->pressure_height != other.pressure_height) {
      return false;
    }
    if (this->magnetic_field != other.magnetic_field) {
      return false;
    }
    if (this->radio_channel != other.radio_channel) {
      return false;
    }
    if (this->seq != other.seq) {
      return false;
    }
    return true;
  }
  bool operator!=(const OutputData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OutputData_

// alias to use template instance with default allocator
using OutputData =
  quadrotor_msgs::msg::OutputData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__STRUCT_HPP_

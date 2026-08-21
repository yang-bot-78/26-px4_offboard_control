// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__STRUCT_HPP_
#define RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__STRUCT_HPP_

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
# define DEPRECATED__race_msgs__msg__FlightAltitudeReference __attribute__((deprecated))
#else
# define DEPRECATED__race_msgs__msg__FlightAltitudeReference __declspec(deprecated)
#endif

namespace race_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FlightAltitudeReference_
{
  using Type = FlightAltitudeReference_<ContainerAllocator>;

  explicit FlightAltitudeReference_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flight_id = 0ull;
      this->valid = false;
      this->target_agl_m = 0.0;
      this->min_agl_m = 0.0;
      this->max_agl_m = 0.0;
      this->ground_z_local_ned = 0.0;
      this->target_z_local_ned = 0.0;
      this->ground_z_map = 0.0;
      this->target_z_map = 0.0;
    }
  }

  explicit FlightAltitudeReference_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flight_id = 0ull;
      this->valid = false;
      this->target_agl_m = 0.0;
      this->min_agl_m = 0.0;
      this->max_agl_m = 0.0;
      this->ground_z_local_ned = 0.0;
      this->target_z_local_ned = 0.0;
      this->ground_z_map = 0.0;
      this->target_z_map = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _flight_id_type =
    uint64_t;
  _flight_id_type flight_id;
  using _valid_type =
    bool;
  _valid_type valid;
  using _target_agl_m_type =
    double;
  _target_agl_m_type target_agl_m;
  using _min_agl_m_type =
    double;
  _min_agl_m_type min_agl_m;
  using _max_agl_m_type =
    double;
  _max_agl_m_type max_agl_m;
  using _ground_z_local_ned_type =
    double;
  _ground_z_local_ned_type ground_z_local_ned;
  using _target_z_local_ned_type =
    double;
  _target_z_local_ned_type target_z_local_ned;
  using _ground_z_map_type =
    double;
  _ground_z_map_type ground_z_map;
  using _target_z_map_type =
    double;
  _target_z_map_type target_z_map;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__flight_id(
    const uint64_t & _arg)
  {
    this->flight_id = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__target_agl_m(
    const double & _arg)
  {
    this->target_agl_m = _arg;
    return *this;
  }
  Type & set__min_agl_m(
    const double & _arg)
  {
    this->min_agl_m = _arg;
    return *this;
  }
  Type & set__max_agl_m(
    const double & _arg)
  {
    this->max_agl_m = _arg;
    return *this;
  }
  Type & set__ground_z_local_ned(
    const double & _arg)
  {
    this->ground_z_local_ned = _arg;
    return *this;
  }
  Type & set__target_z_local_ned(
    const double & _arg)
  {
    this->target_z_local_ned = _arg;
    return *this;
  }
  Type & set__ground_z_map(
    const double & _arg)
  {
    this->ground_z_map = _arg;
    return *this;
  }
  Type & set__target_z_map(
    const double & _arg)
  {
    this->target_z_map = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    race_msgs::msg::FlightAltitudeReference_<ContainerAllocator> *;
  using ConstRawPtr =
    const race_msgs::msg::FlightAltitudeReference_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::FlightAltitudeReference_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::FlightAltitudeReference_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__race_msgs__msg__FlightAltitudeReference
    std::shared_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__race_msgs__msg__FlightAltitudeReference
    std::shared_ptr<race_msgs::msg::FlightAltitudeReference_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FlightAltitudeReference_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->flight_id != other.flight_id) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->target_agl_m != other.target_agl_m) {
      return false;
    }
    if (this->min_agl_m != other.min_agl_m) {
      return false;
    }
    if (this->max_agl_m != other.max_agl_m) {
      return false;
    }
    if (this->ground_z_local_ned != other.ground_z_local_ned) {
      return false;
    }
    if (this->target_z_local_ned != other.target_z_local_ned) {
      return false;
    }
    if (this->ground_z_map != other.ground_z_map) {
      return false;
    }
    if (this->target_z_map != other.target_z_map) {
      return false;
    }
    return true;
  }
  bool operator!=(const FlightAltitudeReference_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FlightAltitudeReference_

// alias to use template instance with default allocator
using FlightAltitudeReference =
  race_msgs::msg::FlightAltitudeReference_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__STRUCT_HPP_

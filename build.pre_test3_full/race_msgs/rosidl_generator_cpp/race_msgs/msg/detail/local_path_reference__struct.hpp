// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from race_msgs:msg/LocalPathReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__STRUCT_HPP_
#define RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__STRUCT_HPP_

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
// Member 'local_goal'
// Member 'points'
// Member 'continuation_points'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__race_msgs__msg__LocalPathReference __attribute__((deprecated))
#else
# define DEPRECATED__race_msgs__msg__LocalPathReference __declspec(deprecated)
#endif

namespace race_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalPathReference_
{
  using Type = LocalPathReference_<ContainerAllocator>;

  explicit LocalPathReference_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    local_goal(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->global_path_id = 0ull;
      this->local_goal_seq = 0ull;
      this->arc_length = 0.0;
      this->minimum_clearance = 0.0;
    }
  }

  explicit LocalPathReference_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    local_goal(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->global_path_id = 0ull;
      this->local_goal_seq = 0ull;
      this->arc_length = 0.0;
      this->minimum_clearance = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _global_path_id_type =
    uint64_t;
  _global_path_id_type global_path_id;
  using _local_goal_seq_type =
    uint64_t;
  _local_goal_seq_type local_goal_seq;
  using _local_goal_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _local_goal_type local_goal;
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;
  using _continuation_points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _continuation_points_type continuation_points;
  using _arc_length_type =
    double;
  _arc_length_type arc_length;
  using _minimum_clearance_type =
    double;
  _minimum_clearance_type minimum_clearance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__global_path_id(
    const uint64_t & _arg)
  {
    this->global_path_id = _arg;
    return *this;
  }
  Type & set__local_goal_seq(
    const uint64_t & _arg)
  {
    this->local_goal_seq = _arg;
    return *this;
  }
  Type & set__local_goal(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->local_goal = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__continuation_points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->continuation_points = _arg;
    return *this;
  }
  Type & set__arc_length(
    const double & _arg)
  {
    this->arc_length = _arg;
    return *this;
  }
  Type & set__minimum_clearance(
    const double & _arg)
  {
    this->minimum_clearance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    race_msgs::msg::LocalPathReference_<ContainerAllocator> *;
  using ConstRawPtr =
    const race_msgs::msg::LocalPathReference_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::LocalPathReference_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      race_msgs::msg::LocalPathReference_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__race_msgs__msg__LocalPathReference
    std::shared_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__race_msgs__msg__LocalPathReference
    std::shared_ptr<race_msgs::msg::LocalPathReference_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalPathReference_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->global_path_id != other.global_path_id) {
      return false;
    }
    if (this->local_goal_seq != other.local_goal_seq) {
      return false;
    }
    if (this->local_goal != other.local_goal) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->continuation_points != other.continuation_points) {
      return false;
    }
    if (this->arc_length != other.arc_length) {
      return false;
    }
    if (this->minimum_clearance != other.minimum_clearance) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalPathReference_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalPathReference_

// alias to use template instance with default allocator
using LocalPathReference =
  race_msgs::msg::LocalPathReference_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__STRUCT_HPP_

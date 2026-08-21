// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from fastlio_global_slam:srv/Relocalize.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__STRUCT_HPP_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__fastlio_global_slam__srv__Relocalize_Request __attribute__((deprecated))
#else
# define DEPRECATED__fastlio_global_slam__srv__Relocalize_Request __declspec(deprecated)
#endif

namespace fastlio_global_slam
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Relocalize_Request_
{
  using Type = Relocalize_Request_<ContainerAllocator>;

  explicit Relocalize_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->use_latest_scan = false;
    }
  }

  explicit Relocalize_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->use_latest_scan = false;
    }
  }

  // field types and members
  using _use_latest_scan_type =
    bool;
  _use_latest_scan_type use_latest_scan;

  // setters for named parameter idiom
  Type & set__use_latest_scan(
    const bool & _arg)
  {
    this->use_latest_scan = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fastlio_global_slam__srv__Relocalize_Request
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fastlio_global_slam__srv__Relocalize_Request
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Relocalize_Request_ & other) const
  {
    if (this->use_latest_scan != other.use_latest_scan) {
      return false;
    }
    return true;
  }
  bool operator!=(const Relocalize_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Relocalize_Request_

// alias to use template instance with default allocator
using Relocalize_Request =
  fastlio_global_slam::srv::Relocalize_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace fastlio_global_slam


// Include directives for member types
// Member 'estimated_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__fastlio_global_slam__srv__Relocalize_Response __attribute__((deprecated))
#else
# define DEPRECATED__fastlio_global_slam__srv__Relocalize_Response __declspec(deprecated)
#endif

namespace fastlio_global_slam
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Relocalize_Response_
{
  using Type = Relocalize_Response_<ContainerAllocator>;

  explicit Relocalize_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : estimated_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Relocalize_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    estimated_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _estimated_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _estimated_pose_type estimated_pose;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__estimated_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->estimated_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fastlio_global_slam__srv__Relocalize_Response
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fastlio_global_slam__srv__Relocalize_Response
    std::shared_ptr<fastlio_global_slam::srv::Relocalize_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Relocalize_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->estimated_pose != other.estimated_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const Relocalize_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Relocalize_Response_

// alias to use template instance with default allocator
using Relocalize_Response =
  fastlio_global_slam::srv::Relocalize_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace fastlio_global_slam

namespace fastlio_global_slam
{

namespace srv
{

struct Relocalize
{
  using Request = fastlio_global_slam::srv::Relocalize_Request;
  using Response = fastlio_global_slam::srv::Relocalize_Response;
};

}  // namespace srv

}  // namespace fastlio_global_slam

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__STRUCT_HPP_

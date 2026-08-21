// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from fastlio_global_slam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__STRUCT_HPP_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__fastlio_global_slam__srv__SaveMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__fastlio_global_slam__srv__SaveMap_Request __declspec(deprecated)
#endif

namespace fastlio_global_slam
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveMap_Request_
{
  using Type = SaveMap_Request_<ContainerAllocator>;

  explicit SaveMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->directory = "";
      this->resolution = 0.0f;
    }
  }

  explicit SaveMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : directory(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->directory = "";
      this->resolution = 0.0f;
    }
  }

  // field types and members
  using _directory_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _directory_type directory;
  using _resolution_type =
    float;
  _resolution_type resolution;

  // setters for named parameter idiom
  Type & set__directory(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->directory = _arg;
    return *this;
  }
  Type & set__resolution(
    const float & _arg)
  {
    this->resolution = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fastlio_global_slam__srv__SaveMap_Request
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fastlio_global_slam__srv__SaveMap_Request
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveMap_Request_ & other) const
  {
    if (this->directory != other.directory) {
      return false;
    }
    if (this->resolution != other.resolution) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveMap_Request_

// alias to use template instance with default allocator
using SaveMap_Request =
  fastlio_global_slam::srv::SaveMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace fastlio_global_slam


#ifndef _WIN32
# define DEPRECATED__fastlio_global_slam__srv__SaveMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__fastlio_global_slam__srv__SaveMap_Response __declspec(deprecated)
#endif

namespace fastlio_global_slam
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SaveMap_Response_
{
  using Type = SaveMap_Response_<ContainerAllocator>;

  explicit SaveMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SaveMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
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

  // constant declarations

  // pointer types
  using RawPtr =
    fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fastlio_global_slam__srv__SaveMap_Response
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fastlio_global_slam__srv__SaveMap_Response
    std::shared_ptr<fastlio_global_slam::srv::SaveMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SaveMap_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SaveMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SaveMap_Response_

// alias to use template instance with default allocator
using SaveMap_Response =
  fastlio_global_slam::srv::SaveMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace fastlio_global_slam

namespace fastlio_global_slam
{

namespace srv
{

struct SaveMap
{
  using Request = fastlio_global_slam::srv::SaveMap_Request;
  using Response = fastlio_global_slam::srv::SaveMap_Response;
};

}  // namespace srv

}  // namespace fastlio_global_slam

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__STRUCT_HPP_

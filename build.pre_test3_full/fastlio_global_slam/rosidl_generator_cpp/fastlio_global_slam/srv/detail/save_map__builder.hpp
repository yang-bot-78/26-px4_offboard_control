// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from fastlio_global_slam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "fastlio_global_slam/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace fastlio_global_slam
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Request_resolution
{
public:
  explicit Init_SaveMap_Request_resolution(::fastlio_global_slam::srv::SaveMap_Request & msg)
  : msg_(msg)
  {}
  ::fastlio_global_slam::srv::SaveMap_Request resolution(::fastlio_global_slam::srv::SaveMap_Request::_resolution_type arg)
  {
    msg_.resolution = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fastlio_global_slam::srv::SaveMap_Request msg_;
};

class Init_SaveMap_Request_directory
{
public:
  Init_SaveMap_Request_directory()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveMap_Request_resolution directory(::fastlio_global_slam::srv::SaveMap_Request::_directory_type arg)
  {
    msg_.directory = std::move(arg);
    return Init_SaveMap_Request_resolution(msg_);
  }

private:
  ::fastlio_global_slam::srv::SaveMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::fastlio_global_slam::srv::SaveMap_Request>()
{
  return fastlio_global_slam::srv::builder::Init_SaveMap_Request_directory();
}

}  // namespace fastlio_global_slam


namespace fastlio_global_slam
{

namespace srv
{

namespace builder
{

class Init_SaveMap_Response_message
{
public:
  explicit Init_SaveMap_Response_message(::fastlio_global_slam::srv::SaveMap_Response & msg)
  : msg_(msg)
  {}
  ::fastlio_global_slam::srv::SaveMap_Response message(::fastlio_global_slam::srv::SaveMap_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fastlio_global_slam::srv::SaveMap_Response msg_;
};

class Init_SaveMap_Response_success
{
public:
  Init_SaveMap_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SaveMap_Response_message success(::fastlio_global_slam::srv::SaveMap_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SaveMap_Response_message(msg_);
  }

private:
  ::fastlio_global_slam::srv::SaveMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::fastlio_global_slam::srv::SaveMap_Response>()
{
  return fastlio_global_slam::srv::builder::Init_SaveMap_Response_success();
}

}  // namespace fastlio_global_slam

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__BUILDER_HPP_

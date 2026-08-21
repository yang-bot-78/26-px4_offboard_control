// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from fastlio_global_slam:srv/Relocalize.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__BUILDER_HPP_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "fastlio_global_slam/srv/detail/relocalize__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace fastlio_global_slam
{

namespace srv
{

namespace builder
{

class Init_Relocalize_Request_use_latest_scan
{
public:
  Init_Relocalize_Request_use_latest_scan()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::fastlio_global_slam::srv::Relocalize_Request use_latest_scan(::fastlio_global_slam::srv::Relocalize_Request::_use_latest_scan_type arg)
  {
    msg_.use_latest_scan = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fastlio_global_slam::srv::Relocalize_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::fastlio_global_slam::srv::Relocalize_Request>()
{
  return fastlio_global_slam::srv::builder::Init_Relocalize_Request_use_latest_scan();
}

}  // namespace fastlio_global_slam


namespace fastlio_global_slam
{

namespace srv
{

namespace builder
{

class Init_Relocalize_Response_estimated_pose
{
public:
  explicit Init_Relocalize_Response_estimated_pose(::fastlio_global_slam::srv::Relocalize_Response & msg)
  : msg_(msg)
  {}
  ::fastlio_global_slam::srv::Relocalize_Response estimated_pose(::fastlio_global_slam::srv::Relocalize_Response::_estimated_pose_type arg)
  {
    msg_.estimated_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fastlio_global_slam::srv::Relocalize_Response msg_;
};

class Init_Relocalize_Response_message
{
public:
  explicit Init_Relocalize_Response_message(::fastlio_global_slam::srv::Relocalize_Response & msg)
  : msg_(msg)
  {}
  Init_Relocalize_Response_estimated_pose message(::fastlio_global_slam::srv::Relocalize_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_Relocalize_Response_estimated_pose(msg_);
  }

private:
  ::fastlio_global_slam::srv::Relocalize_Response msg_;
};

class Init_Relocalize_Response_success
{
public:
  Init_Relocalize_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Relocalize_Response_message success(::fastlio_global_slam::srv::Relocalize_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Relocalize_Response_message(msg_);
  }

private:
  ::fastlio_global_slam::srv::Relocalize_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::fastlio_global_slam::srv::Relocalize_Response>()
{
  return fastlio_global_slam::srv::builder::Init_Relocalize_Response_success();
}

}  // namespace fastlio_global_slam

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__BUILDER_HPP_

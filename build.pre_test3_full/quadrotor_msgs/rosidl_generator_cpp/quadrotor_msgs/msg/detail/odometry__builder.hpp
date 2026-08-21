// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/odometry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_Odometry_status
{
public:
  explicit Init_Odometry_status(::quadrotor_msgs::msg::Odometry & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::Odometry status(::quadrotor_msgs::msg::Odometry::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::Odometry msg_;
};

class Init_Odometry_kfid
{
public:
  explicit Init_Odometry_kfid(::quadrotor_msgs::msg::Odometry & msg)
  : msg_(msg)
  {}
  Init_Odometry_status kfid(::quadrotor_msgs::msg::Odometry::_kfid_type arg)
  {
    msg_.kfid = std::move(arg);
    return Init_Odometry_status(msg_);
  }

private:
  ::quadrotor_msgs::msg::Odometry msg_;
};

class Init_Odometry_kfodom
{
public:
  explicit Init_Odometry_kfodom(::quadrotor_msgs::msg::Odometry & msg)
  : msg_(msg)
  {}
  Init_Odometry_kfid kfodom(::quadrotor_msgs::msg::Odometry::_kfodom_type arg)
  {
    msg_.kfodom = std::move(arg);
    return Init_Odometry_kfid(msg_);
  }

private:
  ::quadrotor_msgs::msg::Odometry msg_;
};

class Init_Odometry_curodom
{
public:
  Init_Odometry_curodom()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Odometry_kfodom curodom(::quadrotor_msgs::msg::Odometry::_curodom_type arg)
  {
    msg_.curodom = std::move(arg);
    return Init_Odometry_kfodom(msg_);
  }

private:
  ::quadrotor_msgs::msg::Odometry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::Odometry>()
{
  return quadrotor_msgs::msg::builder::Init_Odometry_curodom();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__BUILDER_HPP_

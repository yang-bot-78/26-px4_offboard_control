// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from race_msgs:msg/NavigationSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__BUILDER_HPP_
#define RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "race_msgs/msg/detail/navigation_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace race_msgs
{

namespace msg
{

namespace builder
{

class Init_NavigationSetpoint_yaw_rate
{
public:
  explicit Init_NavigationSetpoint_yaw_rate(::race_msgs::msg::NavigationSetpoint & msg)
  : msg_(msg)
  {}
  ::race_msgs::msg::NavigationSetpoint yaw_rate(::race_msgs::msg::NavigationSetpoint::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::race_msgs::msg::NavigationSetpoint msg_;
};

class Init_NavigationSetpoint_yaw_rate_valid
{
public:
  explicit Init_NavigationSetpoint_yaw_rate_valid(::race_msgs::msg::NavigationSetpoint & msg)
  : msg_(msg)
  {}
  Init_NavigationSetpoint_yaw_rate yaw_rate_valid(::race_msgs::msg::NavigationSetpoint::_yaw_rate_valid_type arg)
  {
    msg_.yaw_rate_valid = std::move(arg);
    return Init_NavigationSetpoint_yaw_rate(msg_);
  }

private:
  ::race_msgs::msg::NavigationSetpoint msg_;
};

class Init_NavigationSetpoint_yaw
{
public:
  explicit Init_NavigationSetpoint_yaw(::race_msgs::msg::NavigationSetpoint & msg)
  : msg_(msg)
  {}
  Init_NavigationSetpoint_yaw_rate_valid yaw(::race_msgs::msg::NavigationSetpoint::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_NavigationSetpoint_yaw_rate_valid(msg_);
  }

private:
  ::race_msgs::msg::NavigationSetpoint msg_;
};

class Init_NavigationSetpoint_velocity
{
public:
  explicit Init_NavigationSetpoint_velocity(::race_msgs::msg::NavigationSetpoint & msg)
  : msg_(msg)
  {}
  Init_NavigationSetpoint_yaw velocity(::race_msgs::msg::NavigationSetpoint::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_NavigationSetpoint_yaw(msg_);
  }

private:
  ::race_msgs::msg::NavigationSetpoint msg_;
};

class Init_NavigationSetpoint_velocity_valid
{
public:
  explicit Init_NavigationSetpoint_velocity_valid(::race_msgs::msg::NavigationSetpoint & msg)
  : msg_(msg)
  {}
  Init_NavigationSetpoint_velocity velocity_valid(::race_msgs::msg::NavigationSetpoint::_velocity_valid_type arg)
  {
    msg_.velocity_valid = std::move(arg);
    return Init_NavigationSetpoint_velocity(msg_);
  }

private:
  ::race_msgs::msg::NavigationSetpoint msg_;
};

class Init_NavigationSetpoint_position
{
public:
  explicit Init_NavigationSetpoint_position(::race_msgs::msg::NavigationSetpoint & msg)
  : msg_(msg)
  {}
  Init_NavigationSetpoint_velocity_valid position(::race_msgs::msg::NavigationSetpoint::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_NavigationSetpoint_velocity_valid(msg_);
  }

private:
  ::race_msgs::msg::NavigationSetpoint msg_;
};

class Init_NavigationSetpoint_header
{
public:
  Init_NavigationSetpoint_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigationSetpoint_position header(::race_msgs::msg::NavigationSetpoint::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NavigationSetpoint_position(msg_);
  }

private:
  ::race_msgs::msg::NavigationSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::race_msgs::msg::NavigationSetpoint>()
{
  return race_msgs::msg::builder::Init_NavigationSetpoint_header();
}

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__BUILDER_HPP_

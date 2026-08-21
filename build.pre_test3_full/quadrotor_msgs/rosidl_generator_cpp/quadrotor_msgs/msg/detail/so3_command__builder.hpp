// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/SO3Command.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/so3_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_SO3Command_aux
{
public:
  explicit Init_SO3Command_aux(::quadrotor_msgs::msg::SO3Command & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::SO3Command aux(::quadrotor_msgs::msg::SO3Command::_aux_type arg)
  {
    msg_.aux = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::SO3Command msg_;
};

class Init_SO3Command_kom
{
public:
  explicit Init_SO3Command_kom(::quadrotor_msgs::msg::SO3Command & msg)
  : msg_(msg)
  {}
  Init_SO3Command_aux kom(::quadrotor_msgs::msg::SO3Command::_kom_type arg)
  {
    msg_.kom = std::move(arg);
    return Init_SO3Command_aux(msg_);
  }

private:
  ::quadrotor_msgs::msg::SO3Command msg_;
};

class Init_SO3Command_kr
{
public:
  explicit Init_SO3Command_kr(::quadrotor_msgs::msg::SO3Command & msg)
  : msg_(msg)
  {}
  Init_SO3Command_kom kr(::quadrotor_msgs::msg::SO3Command::_kr_type arg)
  {
    msg_.kr = std::move(arg);
    return Init_SO3Command_kom(msg_);
  }

private:
  ::quadrotor_msgs::msg::SO3Command msg_;
};

class Init_SO3Command_orientation
{
public:
  explicit Init_SO3Command_orientation(::quadrotor_msgs::msg::SO3Command & msg)
  : msg_(msg)
  {}
  Init_SO3Command_kr orientation(::quadrotor_msgs::msg::SO3Command::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_SO3Command_kr(msg_);
  }

private:
  ::quadrotor_msgs::msg::SO3Command msg_;
};

class Init_SO3Command_force
{
public:
  explicit Init_SO3Command_force(::quadrotor_msgs::msg::SO3Command & msg)
  : msg_(msg)
  {}
  Init_SO3Command_orientation force(::quadrotor_msgs::msg::SO3Command::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_SO3Command_orientation(msg_);
  }

private:
  ::quadrotor_msgs::msg::SO3Command msg_;
};

class Init_SO3Command_header
{
public:
  Init_SO3Command_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SO3Command_force header(::quadrotor_msgs::msg::SO3Command::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SO3Command_force(msg_);
  }

private:
  ::quadrotor_msgs::msg::SO3Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::SO3Command>()
{
  return quadrotor_msgs::msg::builder::Init_SO3Command_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__BUILDER_HPP_

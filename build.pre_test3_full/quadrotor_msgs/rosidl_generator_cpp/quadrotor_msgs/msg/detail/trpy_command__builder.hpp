// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/TRPYCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__TRPY_COMMAND__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__TRPY_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/trpy_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_TRPYCommand_aux
{
public:
  explicit Init_TRPYCommand_aux(::quadrotor_msgs::msg::TRPYCommand & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::TRPYCommand aux(::quadrotor_msgs::msg::TRPYCommand::_aux_type arg)
  {
    msg_.aux = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::TRPYCommand msg_;
};

class Init_TRPYCommand_yaw
{
public:
  explicit Init_TRPYCommand_yaw(::quadrotor_msgs::msg::TRPYCommand & msg)
  : msg_(msg)
  {}
  Init_TRPYCommand_aux yaw(::quadrotor_msgs::msg::TRPYCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_TRPYCommand_aux(msg_);
  }

private:
  ::quadrotor_msgs::msg::TRPYCommand msg_;
};

class Init_TRPYCommand_pitch
{
public:
  explicit Init_TRPYCommand_pitch(::quadrotor_msgs::msg::TRPYCommand & msg)
  : msg_(msg)
  {}
  Init_TRPYCommand_yaw pitch(::quadrotor_msgs::msg::TRPYCommand::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_TRPYCommand_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::TRPYCommand msg_;
};

class Init_TRPYCommand_roll
{
public:
  explicit Init_TRPYCommand_roll(::quadrotor_msgs::msg::TRPYCommand & msg)
  : msg_(msg)
  {}
  Init_TRPYCommand_pitch roll(::quadrotor_msgs::msg::TRPYCommand::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_TRPYCommand_pitch(msg_);
  }

private:
  ::quadrotor_msgs::msg::TRPYCommand msg_;
};

class Init_TRPYCommand_thrust
{
public:
  explicit Init_TRPYCommand_thrust(::quadrotor_msgs::msg::TRPYCommand & msg)
  : msg_(msg)
  {}
  Init_TRPYCommand_roll thrust(::quadrotor_msgs::msg::TRPYCommand::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return Init_TRPYCommand_roll(msg_);
  }

private:
  ::quadrotor_msgs::msg::TRPYCommand msg_;
};

class Init_TRPYCommand_header
{
public:
  Init_TRPYCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TRPYCommand_thrust header(::quadrotor_msgs::msg::TRPYCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TRPYCommand_thrust(msg_);
  }

private:
  ::quadrotor_msgs::msg::TRPYCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::TRPYCommand>()
{
  return quadrotor_msgs::msg::builder::Init_TRPYCommand_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__TRPY_COMMAND__BUILDER_HPP_

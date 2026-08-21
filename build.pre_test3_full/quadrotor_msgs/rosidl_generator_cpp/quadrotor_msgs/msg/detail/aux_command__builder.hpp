// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/aux_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_AuxCommand_use_external_yaw
{
public:
  explicit Init_AuxCommand_use_external_yaw(::quadrotor_msgs::msg::AuxCommand & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::AuxCommand use_external_yaw(::quadrotor_msgs::msg::AuxCommand::_use_external_yaw_type arg)
  {
    msg_.use_external_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::AuxCommand msg_;
};

class Init_AuxCommand_enable_motors
{
public:
  explicit Init_AuxCommand_enable_motors(::quadrotor_msgs::msg::AuxCommand & msg)
  : msg_(msg)
  {}
  Init_AuxCommand_use_external_yaw enable_motors(::quadrotor_msgs::msg::AuxCommand::_enable_motors_type arg)
  {
    msg_.enable_motors = std::move(arg);
    return Init_AuxCommand_use_external_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::AuxCommand msg_;
};

class Init_AuxCommand_angle_corrections
{
public:
  explicit Init_AuxCommand_angle_corrections(::quadrotor_msgs::msg::AuxCommand & msg)
  : msg_(msg)
  {}
  Init_AuxCommand_enable_motors angle_corrections(::quadrotor_msgs::msg::AuxCommand::_angle_corrections_type arg)
  {
    msg_.angle_corrections = std::move(arg);
    return Init_AuxCommand_enable_motors(msg_);
  }

private:
  ::quadrotor_msgs::msg::AuxCommand msg_;
};

class Init_AuxCommand_kf_correction
{
public:
  explicit Init_AuxCommand_kf_correction(::quadrotor_msgs::msg::AuxCommand & msg)
  : msg_(msg)
  {}
  Init_AuxCommand_angle_corrections kf_correction(::quadrotor_msgs::msg::AuxCommand::_kf_correction_type arg)
  {
    msg_.kf_correction = std::move(arg);
    return Init_AuxCommand_angle_corrections(msg_);
  }

private:
  ::quadrotor_msgs::msg::AuxCommand msg_;
};

class Init_AuxCommand_current_yaw
{
public:
  Init_AuxCommand_current_yaw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AuxCommand_kf_correction current_yaw(::quadrotor_msgs::msg::AuxCommand::_current_yaw_type arg)
  {
    msg_.current_yaw = std::move(arg);
    return Init_AuxCommand_kf_correction(msg_);
  }

private:
  ::quadrotor_msgs::msg::AuxCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::AuxCommand>()
{
  return quadrotor_msgs::msg::builder::Init_AuxCommand_current_yaw();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__BUILDER_HPP_

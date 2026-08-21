// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/Corrections.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/corrections__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_Corrections_angle_corrections
{
public:
  explicit Init_Corrections_angle_corrections(::quadrotor_msgs::msg::Corrections & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::Corrections angle_corrections(::quadrotor_msgs::msg::Corrections::_angle_corrections_type arg)
  {
    msg_.angle_corrections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::Corrections msg_;
};

class Init_Corrections_kf_correction
{
public:
  Init_Corrections_kf_correction()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Corrections_angle_corrections kf_correction(::quadrotor_msgs::msg::Corrections::_kf_correction_type arg)
  {
    msg_.kf_correction = std::move(arg);
    return Init_Corrections_angle_corrections(msg_);
  }

private:
  ::quadrotor_msgs::msg::Corrections msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::Corrections>()
{
  return quadrotor_msgs::msg::builder::Init_Corrections_kf_correction();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__BUILDER_HPP_

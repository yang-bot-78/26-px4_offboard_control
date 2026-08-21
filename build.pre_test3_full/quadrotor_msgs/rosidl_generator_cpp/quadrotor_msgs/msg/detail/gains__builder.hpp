// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/Gains.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__GAINS__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__GAINS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/gains__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_Gains_kd_yaw
{
public:
  explicit Init_Gains_kd_yaw(::quadrotor_msgs::msg::Gains & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::Gains kd_yaw(::quadrotor_msgs::msg::Gains::_kd_yaw_type arg)
  {
    msg_.kd_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::Gains msg_;
};

class Init_Gains_kp_yaw
{
public:
  explicit Init_Gains_kp_yaw(::quadrotor_msgs::msg::Gains & msg)
  : msg_(msg)
  {}
  Init_Gains_kd_yaw kp_yaw(::quadrotor_msgs::msg::Gains::_kp_yaw_type arg)
  {
    msg_.kp_yaw = std::move(arg);
    return Init_Gains_kd_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::Gains msg_;
};

class Init_Gains_kd
{
public:
  explicit Init_Gains_kd(::quadrotor_msgs::msg::Gains & msg)
  : msg_(msg)
  {}
  Init_Gains_kp_yaw kd(::quadrotor_msgs::msg::Gains::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_Gains_kp_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::Gains msg_;
};

class Init_Gains_kp
{
public:
  Init_Gains_kp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gains_kd kp(::quadrotor_msgs::msg::Gains::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_Gains_kd(msg_);
  }

private:
  ::quadrotor_msgs::msg::Gains msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::Gains>()
{
  return quadrotor_msgs::msg::builder::Init_Gains_kp();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__GAINS__BUILDER_HPP_

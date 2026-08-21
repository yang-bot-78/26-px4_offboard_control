// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/StatusData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/status_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_StatusData_seq
{
public:
  explicit Init_StatusData_seq(::quadrotor_msgs::msg::StatusData & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::StatusData seq(::quadrotor_msgs::msg::StatusData::_seq_type arg)
  {
    msg_.seq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::StatusData msg_;
};

class Init_StatusData_voltage
{
public:
  explicit Init_StatusData_voltage(::quadrotor_msgs::msg::StatusData & msg)
  : msg_(msg)
  {}
  Init_StatusData_seq voltage(::quadrotor_msgs::msg::StatusData::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_StatusData_seq(msg_);
  }

private:
  ::quadrotor_msgs::msg::StatusData msg_;
};

class Init_StatusData_loop_rate
{
public:
  explicit Init_StatusData_loop_rate(::quadrotor_msgs::msg::StatusData & msg)
  : msg_(msg)
  {}
  Init_StatusData_voltage loop_rate(::quadrotor_msgs::msg::StatusData::_loop_rate_type arg)
  {
    msg_.loop_rate = std::move(arg);
    return Init_StatusData_voltage(msg_);
  }

private:
  ::quadrotor_msgs::msg::StatusData msg_;
};

class Init_StatusData_header
{
public:
  Init_StatusData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StatusData_loop_rate header(::quadrotor_msgs::msg::StatusData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StatusData_loop_rate(msg_);
  }

private:
  ::quadrotor_msgs::msg::StatusData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::StatusData>()
{
  return quadrotor_msgs::msg::builder::Init_StatusData_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__BUILDER_HPP_

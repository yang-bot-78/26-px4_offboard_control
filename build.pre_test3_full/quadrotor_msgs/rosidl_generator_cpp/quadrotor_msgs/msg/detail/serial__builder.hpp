// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/Serial.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__SERIAL__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__SERIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/serial__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_Serial_data
{
public:
  explicit Init_Serial_data(::quadrotor_msgs::msg::Serial & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::Serial data(::quadrotor_msgs::msg::Serial::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::Serial msg_;
};

class Init_Serial_type
{
public:
  explicit Init_Serial_type(::quadrotor_msgs::msg::Serial & msg)
  : msg_(msg)
  {}
  Init_Serial_data type(::quadrotor_msgs::msg::Serial::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Serial_data(msg_);
  }

private:
  ::quadrotor_msgs::msg::Serial msg_;
};

class Init_Serial_channel
{
public:
  explicit Init_Serial_channel(::quadrotor_msgs::msg::Serial & msg)
  : msg_(msg)
  {}
  Init_Serial_type channel(::quadrotor_msgs::msg::Serial::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_Serial_type(msg_);
  }

private:
  ::quadrotor_msgs::msg::Serial msg_;
};

class Init_Serial_header
{
public:
  Init_Serial_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Serial_channel header(::quadrotor_msgs::msg::Serial::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Serial_channel(msg_);
  }

private:
  ::quadrotor_msgs::msg::Serial msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::Serial>()
{
  return quadrotor_msgs::msg::builder::Init_Serial_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__SERIAL__BUILDER_HPP_

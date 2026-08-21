// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/OutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/output_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_OutputData_seq
{
public:
  explicit Init_OutputData_seq(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::OutputData seq(::quadrotor_msgs::msg::OutputData::_seq_type arg)
  {
    msg_.seq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_radio_channel
{
public:
  explicit Init_OutputData_radio_channel(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_seq radio_channel(::quadrotor_msgs::msg::OutputData::_radio_channel_type arg)
  {
    msg_.radio_channel = std::move(arg);
    return Init_OutputData_seq(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_magnetic_field
{
public:
  explicit Init_OutputData_magnetic_field(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_radio_channel magnetic_field(::quadrotor_msgs::msg::OutputData::_magnetic_field_type arg)
  {
    msg_.magnetic_field = std::move(arg);
    return Init_OutputData_radio_channel(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_pressure_height
{
public:
  explicit Init_OutputData_pressure_height(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_magnetic_field pressure_height(::quadrotor_msgs::msg::OutputData::_pressure_height_type arg)
  {
    msg_.pressure_height = std::move(arg);
    return Init_OutputData_magnetic_field(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_pressure_dheight
{
public:
  explicit Init_OutputData_pressure_dheight(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_pressure_height pressure_dheight(::quadrotor_msgs::msg::OutputData::_pressure_dheight_type arg)
  {
    msg_.pressure_dheight = std::move(arg);
    return Init_OutputData_pressure_height(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_linear_acceleration
{
public:
  explicit Init_OutputData_linear_acceleration(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_pressure_dheight linear_acceleration(::quadrotor_msgs::msg::OutputData::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_OutputData_pressure_dheight(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_angular_velocity
{
public:
  explicit Init_OutputData_angular_velocity(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_linear_acceleration angular_velocity(::quadrotor_msgs::msg::OutputData::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_OutputData_linear_acceleration(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_orientation
{
public:
  explicit Init_OutputData_orientation(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_angular_velocity orientation(::quadrotor_msgs::msg::OutputData::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_OutputData_angular_velocity(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_voltage
{
public:
  explicit Init_OutputData_voltage(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_orientation voltage(::quadrotor_msgs::msg::OutputData::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_OutputData_orientation(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_loop_rate
{
public:
  explicit Init_OutputData_loop_rate(::quadrotor_msgs::msg::OutputData & msg)
  : msg_(msg)
  {}
  Init_OutputData_voltage loop_rate(::quadrotor_msgs::msg::OutputData::_loop_rate_type arg)
  {
    msg_.loop_rate = std::move(arg);
    return Init_OutputData_voltage(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

class Init_OutputData_header
{
public:
  Init_OutputData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OutputData_loop_rate header(::quadrotor_msgs::msg::OutputData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OutputData_loop_rate(msg_);
  }

private:
  ::quadrotor_msgs::msg::OutputData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::OutputData>()
{
  return quadrotor_msgs::msg::builder::Init_OutputData_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__BUILDER_HPP_

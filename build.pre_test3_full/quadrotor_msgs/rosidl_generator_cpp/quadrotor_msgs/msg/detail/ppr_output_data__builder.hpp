// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/PPROutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/ppr_output_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_PPROutputData_pwm
{
public:
  explicit Init_PPROutputData_pwm(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::PPROutputData pwm(::quadrotor_msgs::msg::PPROutputData::_pwm_type arg)
  {
    msg_.pwm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_acc_z
{
public:
  explicit Init_PPROutputData_est_acc_z(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_pwm est_acc_z(::quadrotor_msgs::msg::PPROutputData::_est_acc_z_type arg)
  {
    msg_.est_acc_z = std::move(arg);
    return Init_PPROutputData_pwm(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_acc_y
{
public:
  explicit Init_PPROutputData_est_acc_y(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_acc_z est_acc_y(::quadrotor_msgs::msg::PPROutputData::_est_acc_y_type arg)
  {
    msg_.est_acc_y = std::move(arg);
    return Init_PPROutputData_est_acc_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_acc_x
{
public:
  explicit Init_PPROutputData_est_acc_x(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_acc_y est_acc_x(::quadrotor_msgs::msg::PPROutputData::_est_acc_x_type arg)
  {
    msg_.est_acc_x = std::move(arg);
    return Init_PPROutputData_est_acc_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_angvel_z
{
public:
  explicit Init_PPROutputData_est_angvel_z(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_acc_x est_angvel_z(::quadrotor_msgs::msg::PPROutputData::_est_angvel_z_type arg)
  {
    msg_.est_angvel_z = std::move(arg);
    return Init_PPROutputData_est_acc_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_angvel_y
{
public:
  explicit Init_PPROutputData_est_angvel_y(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_angvel_z est_angvel_y(::quadrotor_msgs::msg::PPROutputData::_est_angvel_y_type arg)
  {
    msg_.est_angvel_y = std::move(arg);
    return Init_PPROutputData_est_angvel_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_angvel_x
{
public:
  explicit Init_PPROutputData_est_angvel_x(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_angvel_y est_angvel_x(::quadrotor_msgs::msg::PPROutputData::_est_angvel_x_type arg)
  {
    msg_.est_angvel_x = std::move(arg);
    return Init_PPROutputData_est_angvel_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_yaw
{
public:
  explicit Init_PPROutputData_est_yaw(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_angvel_x est_yaw(::quadrotor_msgs::msg::PPROutputData::_est_yaw_type arg)
  {
    msg_.est_yaw = std::move(arg);
    return Init_PPROutputData_est_angvel_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_pitch
{
public:
  explicit Init_PPROutputData_est_pitch(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_yaw est_pitch(::quadrotor_msgs::msg::PPROutputData::_est_pitch_type arg)
  {
    msg_.est_pitch = std::move(arg);
    return Init_PPROutputData_est_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_est_roll
{
public:
  explicit Init_PPROutputData_est_roll(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_pitch est_roll(::quadrotor_msgs::msg::PPROutputData::_est_roll_type arg)
  {
    msg_.est_roll = std::move(arg);
    return Init_PPROutputData_est_pitch(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_des_yaw
{
public:
  explicit Init_PPROutputData_des_yaw(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_est_roll des_yaw(::quadrotor_msgs::msg::PPROutputData::_des_yaw_type arg)
  {
    msg_.des_yaw = std::move(arg);
    return Init_PPROutputData_est_roll(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_des_pitch
{
public:
  explicit Init_PPROutputData_des_pitch(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_des_yaw des_pitch(::quadrotor_msgs::msg::PPROutputData::_des_pitch_type arg)
  {
    msg_.des_pitch = std::move(arg);
    return Init_PPROutputData_des_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_des_roll
{
public:
  explicit Init_PPROutputData_des_roll(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_des_pitch des_roll(::quadrotor_msgs::msg::PPROutputData::_des_roll_type arg)
  {
    msg_.des_roll = std::move(arg);
    return Init_PPROutputData_des_pitch(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_des_thrust
{
public:
  explicit Init_PPROutputData_des_thrust(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_des_roll des_thrust(::quadrotor_msgs::msg::PPROutputData::_des_thrust_type arg)
  {
    msg_.des_thrust = std::move(arg);
    return Init_PPROutputData_des_roll(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_quad_time
{
public:
  explicit Init_PPROutputData_quad_time(::quadrotor_msgs::msg::PPROutputData & msg)
  : msg_(msg)
  {}
  Init_PPROutputData_des_thrust quad_time(::quadrotor_msgs::msg::PPROutputData::_quad_time_type arg)
  {
    msg_.quad_time = std::move(arg);
    return Init_PPROutputData_des_thrust(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

class Init_PPROutputData_header
{
public:
  Init_PPROutputData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PPROutputData_quad_time header(::quadrotor_msgs::msg::PPROutputData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PPROutputData_quad_time(msg_);
  }

private:
  ::quadrotor_msgs::msg::PPROutputData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::PPROutputData>()
{
  return quadrotor_msgs::msg::builder::Init_PPROutputData_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__BUILDER_HPP_

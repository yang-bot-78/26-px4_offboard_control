// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/polynomial_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_PolynomialTrajectory_debug_info
{
public:
  explicit Init_PolynomialTrajectory_debug_info(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::PolynomialTrajectory debug_info(::quadrotor_msgs::msg::PolynomialTrajectory::_debug_info_type arg)
  {
    msg_.debug_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_order
{
public:
  explicit Init_PolynomialTrajectory_order(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_debug_info order(::quadrotor_msgs::msg::PolynomialTrajectory::_order_type arg)
  {
    msg_.order = std::move(arg);
    return Init_PolynomialTrajectory_debug_info(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_mag_coeff
{
public:
  explicit Init_PolynomialTrajectory_mag_coeff(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_order mag_coeff(::quadrotor_msgs::msg::PolynomialTrajectory::_mag_coeff_type arg)
  {
    msg_.mag_coeff = std::move(arg);
    return Init_PolynomialTrajectory_order(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_time
{
public:
  explicit Init_PolynomialTrajectory_time(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_mag_coeff time(::quadrotor_msgs::msg::PolynomialTrajectory::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_PolynomialTrajectory_mag_coeff(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_coef_z
{
public:
  explicit Init_PolynomialTrajectory_coef_z(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_time coef_z(::quadrotor_msgs::msg::PolynomialTrajectory::_coef_z_type arg)
  {
    msg_.coef_z = std::move(arg);
    return Init_PolynomialTrajectory_time(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_coef_y
{
public:
  explicit Init_PolynomialTrajectory_coef_y(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_coef_z coef_y(::quadrotor_msgs::msg::PolynomialTrajectory::_coef_y_type arg)
  {
    msg_.coef_y = std::move(arg);
    return Init_PolynomialTrajectory_coef_z(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_coef_x
{
public:
  explicit Init_PolynomialTrajectory_coef_x(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_coef_y coef_x(::quadrotor_msgs::msg::PolynomialTrajectory::_coef_x_type arg)
  {
    msg_.coef_x = std::move(arg);
    return Init_PolynomialTrajectory_coef_y(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_final_yaw
{
public:
  explicit Init_PolynomialTrajectory_final_yaw(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_coef_x final_yaw(::quadrotor_msgs::msg::PolynomialTrajectory::_final_yaw_type arg)
  {
    msg_.final_yaw = std::move(arg);
    return Init_PolynomialTrajectory_coef_x(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_start_yaw
{
public:
  explicit Init_PolynomialTrajectory_start_yaw(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_final_yaw start_yaw(::quadrotor_msgs::msg::PolynomialTrajectory::_start_yaw_type arg)
  {
    msg_.start_yaw = std::move(arg);
    return Init_PolynomialTrajectory_final_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_num_segment
{
public:
  explicit Init_PolynomialTrajectory_num_segment(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_start_yaw num_segment(::quadrotor_msgs::msg::PolynomialTrajectory::_num_segment_type arg)
  {
    msg_.num_segment = std::move(arg);
    return Init_PolynomialTrajectory_start_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_num_order
{
public:
  explicit Init_PolynomialTrajectory_num_order(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_num_segment num_order(::quadrotor_msgs::msg::PolynomialTrajectory::_num_order_type arg)
  {
    msg_.num_order = std::move(arg);
    return Init_PolynomialTrajectory_num_segment(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_action
{
public:
  explicit Init_PolynomialTrajectory_action(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_num_order action(::quadrotor_msgs::msg::PolynomialTrajectory::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_PolynomialTrajectory_num_order(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_trajectory_id
{
public:
  explicit Init_PolynomialTrajectory_trajectory_id(::quadrotor_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  Init_PolynomialTrajectory_action trajectory_id(::quadrotor_msgs::msg::PolynomialTrajectory::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return Init_PolynomialTrajectory_action(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_header
{
public:
  Init_PolynomialTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolynomialTrajectory_trajectory_id header(::quadrotor_msgs::msg::PolynomialTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PolynomialTrajectory_trajectory_id(msg_);
  }

private:
  ::quadrotor_msgs::msg::PolynomialTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::PolynomialTrajectory>()
{
  return quadrotor_msgs::msg::builder::Init_PolynomialTrajectory_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__BUILDER_HPP_

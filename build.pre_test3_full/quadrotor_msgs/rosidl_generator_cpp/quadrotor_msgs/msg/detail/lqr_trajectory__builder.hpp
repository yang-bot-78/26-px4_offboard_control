// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/LQRTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quadrotor_msgs/msg/detail/lqr_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_LQRTrajectory_debug_info
{
public:
  explicit Init_LQRTrajectory_debug_info(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::LQRTrajectory debug_info(::quadrotor_msgs::msg::LQRTrajectory::_debug_info_type arg)
  {
    msg_.debug_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_t_f
{
public:
  explicit Init_LQRTrajectory_t_f(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_debug_info t_f(::quadrotor_msgs::msg::LQRTrajectory::_t_f_type arg)
  {
    msg_.t_f = std::move(arg);
    return Init_LQRTrajectory_debug_info(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_sf
{
public:
  explicit Init_LQRTrajectory_sf(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_t_f sf(::quadrotor_msgs::msg::LQRTrajectory::_sf_type arg)
  {
    msg_.sf = std::move(arg);
    return Init_LQRTrajectory_t_f(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_ut
{
public:
  explicit Init_LQRTrajectory_ut(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_sf ut(::quadrotor_msgs::msg::LQRTrajectory::_ut_type arg)
  {
    msg_.ut = std::move(arg);
    return Init_LQRTrajectory_sf(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_s0
{
public:
  explicit Init_LQRTrajectory_s0(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_ut s0(::quadrotor_msgs::msg::LQRTrajectory::_s0_type arg)
  {
    msg_.s0 = std::move(arg);
    return Init_LQRTrajectory_ut(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_final_yaw
{
public:
  explicit Init_LQRTrajectory_final_yaw(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_s0 final_yaw(::quadrotor_msgs::msg::LQRTrajectory::_final_yaw_type arg)
  {
    msg_.final_yaw = std::move(arg);
    return Init_LQRTrajectory_s0(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_start_yaw
{
public:
  explicit Init_LQRTrajectory_start_yaw(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_final_yaw start_yaw(::quadrotor_msgs::msg::LQRTrajectory::_start_yaw_type arg)
  {
    msg_.start_yaw = std::move(arg);
    return Init_LQRTrajectory_final_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_r
{
public:
  explicit Init_LQRTrajectory_r(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_start_yaw r(::quadrotor_msgs::msg::LQRTrajectory::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_LQRTrajectory_start_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_action
{
public:
  explicit Init_LQRTrajectory_action(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_r action(::quadrotor_msgs::msg::LQRTrajectory::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_LQRTrajectory_r(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_trajectory_id
{
public:
  explicit Init_LQRTrajectory_trajectory_id(::quadrotor_msgs::msg::LQRTrajectory & msg)
  : msg_(msg)
  {}
  Init_LQRTrajectory_action trajectory_id(::quadrotor_msgs::msg::LQRTrajectory::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return Init_LQRTrajectory_action(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

class Init_LQRTrajectory_header
{
public:
  Init_LQRTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LQRTrajectory_trajectory_id header(::quadrotor_msgs::msg::LQRTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LQRTrajectory_trajectory_id(msg_);
  }

private:
  ::quadrotor_msgs::msg::LQRTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::LQRTrajectory>()
{
  return quadrotor_msgs::msg::builder::Init_LQRTrajectory_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__BUILDER_HPP_

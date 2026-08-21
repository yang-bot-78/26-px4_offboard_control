// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from race_msgs:msg/GlobalPlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__BUILDER_HPP_
#define RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "race_msgs/msg/detail/global_planner_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace race_msgs
{

namespace msg
{

namespace builder
{

class Init_GlobalPlannerStatus_reason
{
public:
  explicit Init_GlobalPlannerStatus_reason(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  ::race_msgs::msg::GlobalPlannerStatus reason(::race_msgs::msg::GlobalPlannerStatus::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_mode
{
public:
  explicit Init_GlobalPlannerStatus_mode(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_reason mode(::race_msgs::msg::GlobalPlannerStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_GlobalPlannerStatus_reason(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_horizontal_speed
{
public:
  explicit Init_GlobalPlannerStatus_horizontal_speed(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_mode horizontal_speed(::race_msgs::msg::GlobalPlannerStatus::_horizontal_speed_type arg)
  {
    msg_.horizontal_speed = std::move(arg);
    return Init_GlobalPlannerStatus_mode(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_distance_to_final
{
public:
  explicit Init_GlobalPlannerStatus_distance_to_final(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_horizontal_speed distance_to_final(::race_msgs::msg::GlobalPlannerStatus::_distance_to_final_type arg)
  {
    msg_.distance_to_final = std::move(arg);
    return Init_GlobalPlannerStatus_horizontal_speed(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_final_goal_reached
{
public:
  explicit Init_GlobalPlannerStatus_final_goal_reached(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_distance_to_final final_goal_reached(::race_msgs::msg::GlobalPlannerStatus::_final_goal_reached_type arg)
  {
    msg_.final_goal_reached = std::move(arg);
    return Init_GlobalPlannerStatus_distance_to_final(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_goal_active
{
public:
  explicit Init_GlobalPlannerStatus_goal_active(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_final_goal_reached goal_active(::race_msgs::msg::GlobalPlannerStatus::_goal_active_type arg)
  {
    msg_.goal_active = std::move(arg);
    return Init_GlobalPlannerStatus_final_goal_reached(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_local_goal_seq
{
public:
  explicit Init_GlobalPlannerStatus_local_goal_seq(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_goal_active local_goal_seq(::race_msgs::msg::GlobalPlannerStatus::_local_goal_seq_type arg)
  {
    msg_.local_goal_seq = std::move(arg);
    return Init_GlobalPlannerStatus_goal_active(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_global_path_id
{
public:
  explicit Init_GlobalPlannerStatus_global_path_id(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_local_goal_seq global_path_id(::race_msgs::msg::GlobalPlannerStatus::_global_path_id_type arg)
  {
    msg_.global_path_id = std::move(arg);
    return Init_GlobalPlannerStatus_local_goal_seq(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_global_goal_id
{
public:
  explicit Init_GlobalPlannerStatus_global_goal_id(::race_msgs::msg::GlobalPlannerStatus & msg)
  : msg_(msg)
  {}
  Init_GlobalPlannerStatus_global_path_id global_goal_id(::race_msgs::msg::GlobalPlannerStatus::_global_goal_id_type arg)
  {
    msg_.global_goal_id = std::move(arg);
    return Init_GlobalPlannerStatus_global_path_id(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

class Init_GlobalPlannerStatus_header
{
public:
  Init_GlobalPlannerStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GlobalPlannerStatus_global_goal_id header(::race_msgs::msg::GlobalPlannerStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GlobalPlannerStatus_global_goal_id(msg_);
  }

private:
  ::race_msgs::msg::GlobalPlannerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::race_msgs::msg::GlobalPlannerStatus>()
{
  return race_msgs::msg::builder::Init_GlobalPlannerStatus_header();
}

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__BUILDER_HPP_

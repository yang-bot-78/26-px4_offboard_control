// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from race_msgs:msg/LocalPathReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__BUILDER_HPP_
#define RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "race_msgs/msg/detail/local_path_reference__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace race_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalPathReference_minimum_clearance
{
public:
  explicit Init_LocalPathReference_minimum_clearance(::race_msgs::msg::LocalPathReference & msg)
  : msg_(msg)
  {}
  ::race_msgs::msg::LocalPathReference minimum_clearance(::race_msgs::msg::LocalPathReference::_minimum_clearance_type arg)
  {
    msg_.minimum_clearance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

class Init_LocalPathReference_arc_length
{
public:
  explicit Init_LocalPathReference_arc_length(::race_msgs::msg::LocalPathReference & msg)
  : msg_(msg)
  {}
  Init_LocalPathReference_minimum_clearance arc_length(::race_msgs::msg::LocalPathReference::_arc_length_type arg)
  {
    msg_.arc_length = std::move(arg);
    return Init_LocalPathReference_minimum_clearance(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

class Init_LocalPathReference_continuation_points
{
public:
  explicit Init_LocalPathReference_continuation_points(::race_msgs::msg::LocalPathReference & msg)
  : msg_(msg)
  {}
  Init_LocalPathReference_arc_length continuation_points(::race_msgs::msg::LocalPathReference::_continuation_points_type arg)
  {
    msg_.continuation_points = std::move(arg);
    return Init_LocalPathReference_arc_length(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

class Init_LocalPathReference_points
{
public:
  explicit Init_LocalPathReference_points(::race_msgs::msg::LocalPathReference & msg)
  : msg_(msg)
  {}
  Init_LocalPathReference_continuation_points points(::race_msgs::msg::LocalPathReference::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_LocalPathReference_continuation_points(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

class Init_LocalPathReference_local_goal
{
public:
  explicit Init_LocalPathReference_local_goal(::race_msgs::msg::LocalPathReference & msg)
  : msg_(msg)
  {}
  Init_LocalPathReference_points local_goal(::race_msgs::msg::LocalPathReference::_local_goal_type arg)
  {
    msg_.local_goal = std::move(arg);
    return Init_LocalPathReference_points(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

class Init_LocalPathReference_local_goal_seq
{
public:
  explicit Init_LocalPathReference_local_goal_seq(::race_msgs::msg::LocalPathReference & msg)
  : msg_(msg)
  {}
  Init_LocalPathReference_local_goal local_goal_seq(::race_msgs::msg::LocalPathReference::_local_goal_seq_type arg)
  {
    msg_.local_goal_seq = std::move(arg);
    return Init_LocalPathReference_local_goal(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

class Init_LocalPathReference_global_path_id
{
public:
  explicit Init_LocalPathReference_global_path_id(::race_msgs::msg::LocalPathReference & msg)
  : msg_(msg)
  {}
  Init_LocalPathReference_local_goal_seq global_path_id(::race_msgs::msg::LocalPathReference::_global_path_id_type arg)
  {
    msg_.global_path_id = std::move(arg);
    return Init_LocalPathReference_local_goal_seq(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

class Init_LocalPathReference_header
{
public:
  Init_LocalPathReference_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPathReference_global_path_id header(::race_msgs::msg::LocalPathReference::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LocalPathReference_global_path_id(msg_);
  }

private:
  ::race_msgs::msg::LocalPathReference msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::race_msgs::msg::LocalPathReference>()
{
  return race_msgs::msg::builder::Init_LocalPathReference_header();
}

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__BUILDER_HPP_

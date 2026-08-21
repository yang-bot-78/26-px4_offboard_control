// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from race_msgs:msg/GlobalPlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__TRAITS_HPP_
#define RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "race_msgs/msg/detail/global_planner_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace race_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GlobalPlannerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: global_goal_id
  {
    out << "global_goal_id: ";
    rosidl_generator_traits::value_to_yaml(msg.global_goal_id, out);
    out << ", ";
  }

  // member: global_path_id
  {
    out << "global_path_id: ";
    rosidl_generator_traits::value_to_yaml(msg.global_path_id, out);
    out << ", ";
  }

  // member: local_goal_seq
  {
    out << "local_goal_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.local_goal_seq, out);
    out << ", ";
  }

  // member: goal_active
  {
    out << "goal_active: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_active, out);
    out << ", ";
  }

  // member: final_goal_reached
  {
    out << "final_goal_reached: ";
    rosidl_generator_traits::value_to_yaml(msg.final_goal_reached, out);
    out << ", ";
  }

  // member: distance_to_final
  {
    out << "distance_to_final: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_final, out);
    out << ", ";
  }

  // member: horizontal_speed
  {
    out << "horizontal_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.horizontal_speed, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GlobalPlannerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: global_goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "global_goal_id: ";
    rosidl_generator_traits::value_to_yaml(msg.global_goal_id, out);
    out << "\n";
  }

  // member: global_path_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "global_path_id: ";
    rosidl_generator_traits::value_to_yaml(msg.global_path_id, out);
    out << "\n";
  }

  // member: local_goal_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_goal_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.local_goal_seq, out);
    out << "\n";
  }

  // member: goal_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_active: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_active, out);
    out << "\n";
  }

  // member: final_goal_reached
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_goal_reached: ";
    rosidl_generator_traits::value_to_yaml(msg.final_goal_reached, out);
    out << "\n";
  }

  // member: distance_to_final
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_to_final: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_final, out);
    out << "\n";
  }

  // member: horizontal_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "horizontal_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.horizontal_speed, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GlobalPlannerStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace race_msgs

namespace rosidl_generator_traits
{

[[deprecated("use race_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const race_msgs::msg::GlobalPlannerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  race_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use race_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const race_msgs::msg::GlobalPlannerStatus & msg)
{
  return race_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<race_msgs::msg::GlobalPlannerStatus>()
{
  return "race_msgs::msg::GlobalPlannerStatus";
}

template<>
inline const char * name<race_msgs::msg::GlobalPlannerStatus>()
{
  return "race_msgs/msg/GlobalPlannerStatus";
}

template<>
struct has_fixed_size<race_msgs::msg::GlobalPlannerStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<race_msgs::msg::GlobalPlannerStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<race_msgs::msg::GlobalPlannerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__TRAITS_HPP_

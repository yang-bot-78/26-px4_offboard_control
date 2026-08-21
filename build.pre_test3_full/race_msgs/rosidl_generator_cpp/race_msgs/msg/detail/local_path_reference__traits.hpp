// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from race_msgs:msg/LocalPathReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__TRAITS_HPP_
#define RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "race_msgs/msg/detail/local_path_reference__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'local_goal'
// Member 'points'
// Member 'continuation_points'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace race_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalPathReference & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
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

  // member: local_goal
  {
    out << "local_goal: ";
    to_flow_style_yaml(msg.local_goal, out);
    out << ", ";
  }

  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: continuation_points
  {
    if (msg.continuation_points.size() == 0) {
      out << "continuation_points: []";
    } else {
      out << "continuation_points: [";
      size_t pending_items = msg.continuation_points.size();
      for (auto item : msg.continuation_points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: arc_length
  {
    out << "arc_length: ";
    rosidl_generator_traits::value_to_yaml(msg.arc_length, out);
    out << ", ";
  }

  // member: minimum_clearance
  {
    out << "minimum_clearance: ";
    rosidl_generator_traits::value_to_yaml(msg.minimum_clearance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalPathReference & msg,
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

  // member: local_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "local_goal:\n";
    to_block_style_yaml(msg.local_goal, out, indentation + 2);
  }

  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: continuation_points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.continuation_points.size() == 0) {
      out << "continuation_points: []\n";
    } else {
      out << "continuation_points:\n";
      for (auto item : msg.continuation_points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: arc_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arc_length: ";
    rosidl_generator_traits::value_to_yaml(msg.arc_length, out);
    out << "\n";
  }

  // member: minimum_clearance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "minimum_clearance: ";
    rosidl_generator_traits::value_to_yaml(msg.minimum_clearance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalPathReference & msg, bool use_flow_style = false)
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
  const race_msgs::msg::LocalPathReference & msg,
  std::ostream & out, size_t indentation = 0)
{
  race_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use race_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const race_msgs::msg::LocalPathReference & msg)
{
  return race_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<race_msgs::msg::LocalPathReference>()
{
  return "race_msgs::msg::LocalPathReference";
}

template<>
inline const char * name<race_msgs::msg::LocalPathReference>()
{
  return "race_msgs/msg/LocalPathReference";
}

template<>
struct has_fixed_size<race_msgs::msg::LocalPathReference>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<race_msgs::msg::LocalPathReference>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<race_msgs::msg::LocalPathReference>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from race_msgs:msg/NavigationSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__TRAITS_HPP_
#define RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "race_msgs/msg/detail/navigation_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace race_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavigationSetpoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity_valid
  {
    out << "velocity_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_valid, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: yaw_rate_valid
  {
    out << "yaw_rate_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate_valid, out);
    out << ", ";
  }

  // member: yaw_rate
  {
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigationSetpoint & msg,
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

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_valid, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: yaw_rate_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate_valid, out);
    out << "\n";
  }

  // member: yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigationSetpoint & msg, bool use_flow_style = false)
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
  const race_msgs::msg::NavigationSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  race_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use race_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const race_msgs::msg::NavigationSetpoint & msg)
{
  return race_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<race_msgs::msg::NavigationSetpoint>()
{
  return "race_msgs::msg::NavigationSetpoint";
}

template<>
inline const char * name<race_msgs::msg::NavigationSetpoint>()
{
  return "race_msgs/msg/NavigationSetpoint";
}

template<>
struct has_fixed_size<race_msgs::msg::NavigationSetpoint>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<race_msgs::msg::NavigationSetpoint>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<race_msgs::msg::NavigationSetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__TRAITS_HPP_

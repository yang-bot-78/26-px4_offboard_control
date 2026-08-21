// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/odometry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'curodom'
// Member 'kfodom'
#include "nav_msgs/msg/detail/odometry__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Odometry & msg,
  std::ostream & out)
{
  out << "{";
  // member: curodom
  {
    out << "curodom: ";
    to_flow_style_yaml(msg.curodom, out);
    out << ", ";
  }

  // member: kfodom
  {
    out << "kfodom: ";
    to_flow_style_yaml(msg.kfodom, out);
    out << ", ";
  }

  // member: kfid
  {
    out << "kfid: ";
    rosidl_generator_traits::value_to_yaml(msg.kfid, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Odometry & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: curodom
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "curodom:\n";
    to_block_style_yaml(msg.curodom, out, indentation + 2);
  }

  // member: kfodom
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kfodom:\n";
    to_block_style_yaml(msg.kfodom, out, indentation + 2);
  }

  // member: kfid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kfid: ";
    rosidl_generator_traits::value_to_yaml(msg.kfid, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Odometry & msg, bool use_flow_style = false)
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

}  // namespace quadrotor_msgs

namespace rosidl_generator_traits
{

[[deprecated("use quadrotor_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const quadrotor_msgs::msg::Odometry & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::Odometry & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::Odometry>()
{
  return "quadrotor_msgs::msg::Odometry";
}

template<>
inline const char * name<quadrotor_msgs::msg::Odometry>()
{
  return "quadrotor_msgs/msg/Odometry";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::Odometry>
  : std::integral_constant<bool, has_fixed_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::Odometry>
  : std::integral_constant<bool, has_bounded_size<nav_msgs::msg::Odometry>::value> {};

template<>
struct is_message<quadrotor_msgs::msg::Odometry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/Corrections.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/corrections__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Corrections & msg,
  std::ostream & out)
{
  out << "{";
  // member: kf_correction
  {
    out << "kf_correction: ";
    rosidl_generator_traits::value_to_yaml(msg.kf_correction, out);
    out << ", ";
  }

  // member: angle_corrections
  {
    if (msg.angle_corrections.size() == 0) {
      out << "angle_corrections: []";
    } else {
      out << "angle_corrections: [";
      size_t pending_items = msg.angle_corrections.size();
      for (auto item : msg.angle_corrections) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Corrections & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: kf_correction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kf_correction: ";
    rosidl_generator_traits::value_to_yaml(msg.kf_correction, out);
    out << "\n";
  }

  // member: angle_corrections
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angle_corrections.size() == 0) {
      out << "angle_corrections: []\n";
    } else {
      out << "angle_corrections:\n";
      for (auto item : msg.angle_corrections) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Corrections & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::Corrections & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::Corrections & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::Corrections>()
{
  return "quadrotor_msgs::msg::Corrections";
}

template<>
inline const char * name<quadrotor_msgs::msg::Corrections>()
{
  return "quadrotor_msgs/msg/Corrections";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::Corrections>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::Corrections>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<quadrotor_msgs::msg::Corrections>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__TRAITS_HPP_

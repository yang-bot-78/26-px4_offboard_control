// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/aux_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AuxCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_yaw
  {
    out << "current_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.current_yaw, out);
    out << ", ";
  }

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
    out << ", ";
  }

  // member: enable_motors
  {
    out << "enable_motors: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_motors, out);
    out << ", ";
  }

  // member: use_external_yaw
  {
    out << "use_external_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.use_external_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AuxCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.current_yaw, out);
    out << "\n";
  }

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

  // member: enable_motors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_motors: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_motors, out);
    out << "\n";
  }

  // member: use_external_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_external_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.use_external_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AuxCommand & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::AuxCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::AuxCommand & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::AuxCommand>()
{
  return "quadrotor_msgs::msg::AuxCommand";
}

template<>
inline const char * name<quadrotor_msgs::msg::AuxCommand>()
{
  return "quadrotor_msgs/msg/AuxCommand";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::AuxCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::AuxCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<quadrotor_msgs::msg::AuxCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__TRAITS_HPP_

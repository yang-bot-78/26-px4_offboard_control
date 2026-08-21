// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/OutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/output_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'angular_velocity'
// Member 'linear_acceleration'
// Member 'magnetic_field'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OutputData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: loop_rate
  {
    out << "loop_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_rate, out);
    out << ", ";
  }

  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    to_flow_style_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: linear_acceleration
  {
    out << "linear_acceleration: ";
    to_flow_style_yaml(msg.linear_acceleration, out);
    out << ", ";
  }

  // member: pressure_dheight
  {
    out << "pressure_dheight: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_dheight, out);
    out << ", ";
  }

  // member: pressure_height
  {
    out << "pressure_height: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_height, out);
    out << ", ";
  }

  // member: magnetic_field
  {
    out << "magnetic_field: ";
    to_flow_style_yaml(msg.magnetic_field, out);
    out << ", ";
  }

  // member: radio_channel
  {
    if (msg.radio_channel.size() == 0) {
      out << "radio_channel: []";
    } else {
      out << "radio_channel: [";
      size_t pending_items = msg.radio_channel.size();
      for (auto item : msg.radio_channel) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: seq
  {
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OutputData & msg,
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

  // member: loop_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loop_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_rate, out);
    out << "\n";
  }

  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_block_style_yaml(msg.angular_velocity, out, indentation + 2);
  }

  // member: linear_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_acceleration:\n";
    to_block_style_yaml(msg.linear_acceleration, out, indentation + 2);
  }

  // member: pressure_dheight
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pressure_dheight: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_dheight, out);
    out << "\n";
  }

  // member: pressure_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pressure_height: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_height, out);
    out << "\n";
  }

  // member: magnetic_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetic_field:\n";
    to_block_style_yaml(msg.magnetic_field, out, indentation + 2);
  }

  // member: radio_channel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.radio_channel.size() == 0) {
      out << "radio_channel: []\n";
    } else {
      out << "radio_channel:\n";
      for (auto item : msg.radio_channel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OutputData & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::OutputData & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::OutputData & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::OutputData>()
{
  return "quadrotor_msgs::msg::OutputData";
}

template<>
inline const char * name<quadrotor_msgs::msg::OutputData>()
{
  return "quadrotor_msgs/msg/OutputData";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::OutputData>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::OutputData>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<quadrotor_msgs::msg::OutputData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__TRAITS_HPP_

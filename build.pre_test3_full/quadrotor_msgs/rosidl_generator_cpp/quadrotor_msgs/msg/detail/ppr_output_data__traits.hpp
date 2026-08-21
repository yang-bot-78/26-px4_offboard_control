// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/PPROutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/ppr_output_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PPROutputData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: quad_time
  {
    out << "quad_time: ";
    rosidl_generator_traits::value_to_yaml(msg.quad_time, out);
    out << ", ";
  }

  // member: des_thrust
  {
    out << "des_thrust: ";
    rosidl_generator_traits::value_to_yaml(msg.des_thrust, out);
    out << ", ";
  }

  // member: des_roll
  {
    out << "des_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.des_roll, out);
    out << ", ";
  }

  // member: des_pitch
  {
    out << "des_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.des_pitch, out);
    out << ", ";
  }

  // member: des_yaw
  {
    out << "des_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.des_yaw, out);
    out << ", ";
  }

  // member: est_roll
  {
    out << "est_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.est_roll, out);
    out << ", ";
  }

  // member: est_pitch
  {
    out << "est_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.est_pitch, out);
    out << ", ";
  }

  // member: est_yaw
  {
    out << "est_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.est_yaw, out);
    out << ", ";
  }

  // member: est_angvel_x
  {
    out << "est_angvel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.est_angvel_x, out);
    out << ", ";
  }

  // member: est_angvel_y
  {
    out << "est_angvel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.est_angvel_y, out);
    out << ", ";
  }

  // member: est_angvel_z
  {
    out << "est_angvel_z: ";
    rosidl_generator_traits::value_to_yaml(msg.est_angvel_z, out);
    out << ", ";
  }

  // member: est_acc_x
  {
    out << "est_acc_x: ";
    rosidl_generator_traits::value_to_yaml(msg.est_acc_x, out);
    out << ", ";
  }

  // member: est_acc_y
  {
    out << "est_acc_y: ";
    rosidl_generator_traits::value_to_yaml(msg.est_acc_y, out);
    out << ", ";
  }

  // member: est_acc_z
  {
    out << "est_acc_z: ";
    rosidl_generator_traits::value_to_yaml(msg.est_acc_z, out);
    out << ", ";
  }

  // member: pwm
  {
    if (msg.pwm.size() == 0) {
      out << "pwm: []";
    } else {
      out << "pwm: [";
      size_t pending_items = msg.pwm.size();
      for (auto item : msg.pwm) {
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
  const PPROutputData & msg,
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

  // member: quad_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quad_time: ";
    rosidl_generator_traits::value_to_yaml(msg.quad_time, out);
    out << "\n";
  }

  // member: des_thrust
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_thrust: ";
    rosidl_generator_traits::value_to_yaml(msg.des_thrust, out);
    out << "\n";
  }

  // member: des_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.des_roll, out);
    out << "\n";
  }

  // member: des_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.des_pitch, out);
    out << "\n";
  }

  // member: des_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "des_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.des_yaw, out);
    out << "\n";
  }

  // member: est_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.est_roll, out);
    out << "\n";
  }

  // member: est_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.est_pitch, out);
    out << "\n";
  }

  // member: est_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.est_yaw, out);
    out << "\n";
  }

  // member: est_angvel_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_angvel_x: ";
    rosidl_generator_traits::value_to_yaml(msg.est_angvel_x, out);
    out << "\n";
  }

  // member: est_angvel_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_angvel_y: ";
    rosidl_generator_traits::value_to_yaml(msg.est_angvel_y, out);
    out << "\n";
  }

  // member: est_angvel_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_angvel_z: ";
    rosidl_generator_traits::value_to_yaml(msg.est_angvel_z, out);
    out << "\n";
  }

  // member: est_acc_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_acc_x: ";
    rosidl_generator_traits::value_to_yaml(msg.est_acc_x, out);
    out << "\n";
  }

  // member: est_acc_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_acc_y: ";
    rosidl_generator_traits::value_to_yaml(msg.est_acc_y, out);
    out << "\n";
  }

  // member: est_acc_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "est_acc_z: ";
    rosidl_generator_traits::value_to_yaml(msg.est_acc_z, out);
    out << "\n";
  }

  // member: pwm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pwm.size() == 0) {
      out << "pwm: []\n";
    } else {
      out << "pwm:\n";
      for (auto item : msg.pwm) {
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

inline std::string to_yaml(const PPROutputData & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::PPROutputData & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::PPROutputData & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::PPROutputData>()
{
  return "quadrotor_msgs::msg::PPROutputData";
}

template<>
inline const char * name<quadrotor_msgs::msg::PPROutputData>()
{
  return "quadrotor_msgs/msg/PPROutputData";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::PPROutputData>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::PPROutputData>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<quadrotor_msgs::msg::PPROutputData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__TRAITS_HPP_

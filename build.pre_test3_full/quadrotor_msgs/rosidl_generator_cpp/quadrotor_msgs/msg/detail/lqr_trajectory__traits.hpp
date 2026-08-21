// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/LQRTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/lqr_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LQRTrajectory & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: trajectory_id
  {
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << ", ";
  }

  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << ", ";
  }

  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: start_yaw
  {
    out << "start_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.start_yaw, out);
    out << ", ";
  }

  // member: final_yaw
  {
    out << "final_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.final_yaw, out);
    out << ", ";
  }

  // member: s0
  {
    if (msg.s0.size() == 0) {
      out << "s0: []";
    } else {
      out << "s0: [";
      size_t pending_items = msg.s0.size();
      for (auto item : msg.s0) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ut
  {
    if (msg.ut.size() == 0) {
      out << "ut: []";
    } else {
      out << "ut: [";
      size_t pending_items = msg.ut.size();
      for (auto item : msg.ut) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: sf
  {
    if (msg.sf.size() == 0) {
      out << "sf: []";
    } else {
      out << "sf: [";
      size_t pending_items = msg.sf.size();
      for (auto item : msg.sf) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: t_f
  {
    out << "t_f: ";
    rosidl_generator_traits::value_to_yaml(msg.t_f, out);
    out << ", ";
  }

  // member: debug_info
  {
    out << "debug_info: ";
    rosidl_generator_traits::value_to_yaml(msg.debug_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LQRTrajectory & msg,
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

  // member: trajectory_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << "\n";
  }

  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: start_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.start_yaw, out);
    out << "\n";
  }

  // member: final_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.final_yaw, out);
    out << "\n";
  }

  // member: s0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.s0.size() == 0) {
      out << "s0: []\n";
    } else {
      out << "s0:\n";
      for (auto item : msg.s0) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ut
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ut.size() == 0) {
      out << "ut: []\n";
    } else {
      out << "ut:\n";
      for (auto item : msg.ut) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: sf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sf.size() == 0) {
      out << "sf: []\n";
    } else {
      out << "sf:\n";
      for (auto item : msg.sf) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: t_f
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_f: ";
    rosidl_generator_traits::value_to_yaml(msg.t_f, out);
    out << "\n";
  }

  // member: debug_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "debug_info: ";
    rosidl_generator_traits::value_to_yaml(msg.debug_info, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LQRTrajectory & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::LQRTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::LQRTrajectory & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::LQRTrajectory>()
{
  return "quadrotor_msgs::msg::LQRTrajectory";
}

template<>
inline const char * name<quadrotor_msgs::msg::LQRTrajectory>()
{
  return "quadrotor_msgs/msg/LQRTrajectory";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::LQRTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::LQRTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quadrotor_msgs::msg::LQRTrajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__TRAITS_HPP_

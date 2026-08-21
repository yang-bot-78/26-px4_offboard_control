// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quadrotor_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__TRAITS_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quadrotor_msgs/msg/detail/polynomial_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace quadrotor_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PolynomialTrajectory & msg,
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

  // member: num_order
  {
    out << "num_order: ";
    rosidl_generator_traits::value_to_yaml(msg.num_order, out);
    out << ", ";
  }

  // member: num_segment
  {
    out << "num_segment: ";
    rosidl_generator_traits::value_to_yaml(msg.num_segment, out);
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

  // member: coef_x
  {
    if (msg.coef_x.size() == 0) {
      out << "coef_x: []";
    } else {
      out << "coef_x: [";
      size_t pending_items = msg.coef_x.size();
      for (auto item : msg.coef_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: coef_y
  {
    if (msg.coef_y.size() == 0) {
      out << "coef_y: []";
    } else {
      out << "coef_y: [";
      size_t pending_items = msg.coef_y.size();
      for (auto item : msg.coef_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: coef_z
  {
    if (msg.coef_z.size() == 0) {
      out << "coef_z: []";
    } else {
      out << "coef_z: [";
      size_t pending_items = msg.coef_z.size();
      for (auto item : msg.coef_z) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: time
  {
    if (msg.time.size() == 0) {
      out << "time: []";
    } else {
      out << "time: [";
      size_t pending_items = msg.time.size();
      for (auto item : msg.time) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mag_coeff
  {
    out << "mag_coeff: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_coeff, out);
    out << ", ";
  }

  // member: order
  {
    if (msg.order.size() == 0) {
      out << "order: []";
    } else {
      out << "order: [";
      size_t pending_items = msg.order.size();
      for (auto item : msg.order) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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
  const PolynomialTrajectory & msg,
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

  // member: num_order
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_order: ";
    rosidl_generator_traits::value_to_yaml(msg.num_order, out);
    out << "\n";
  }

  // member: num_segment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_segment: ";
    rosidl_generator_traits::value_to_yaml(msg.num_segment, out);
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

  // member: coef_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coef_x.size() == 0) {
      out << "coef_x: []\n";
    } else {
      out << "coef_x:\n";
      for (auto item : msg.coef_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: coef_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coef_y.size() == 0) {
      out << "coef_y: []\n";
    } else {
      out << "coef_y:\n";
      for (auto item : msg.coef_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: coef_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coef_z.size() == 0) {
      out << "coef_z: []\n";
    } else {
      out << "coef_z:\n";
      for (auto item : msg.coef_z) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.time.size() == 0) {
      out << "time: []\n";
    } else {
      out << "time:\n";
      for (auto item : msg.time) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: mag_coeff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mag_coeff: ";
    rosidl_generator_traits::value_to_yaml(msg.mag_coeff, out);
    out << "\n";
  }

  // member: order
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.order.size() == 0) {
      out << "order: []\n";
    } else {
      out << "order:\n";
      for (auto item : msg.order) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

inline std::string to_yaml(const PolynomialTrajectory & msg, bool use_flow_style = false)
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
  const quadrotor_msgs::msg::PolynomialTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  quadrotor_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quadrotor_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quadrotor_msgs::msg::PolynomialTrajectory & msg)
{
  return quadrotor_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quadrotor_msgs::msg::PolynomialTrajectory>()
{
  return "quadrotor_msgs::msg::PolynomialTrajectory";
}

template<>
inline const char * name<quadrotor_msgs::msg::PolynomialTrajectory>()
{
  return "quadrotor_msgs/msg/PolynomialTrajectory";
}

template<>
struct has_fixed_size<quadrotor_msgs::msg::PolynomialTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quadrotor_msgs::msg::PolynomialTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quadrotor_msgs::msg::PolynomialTrajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__TRAITS_HPP_

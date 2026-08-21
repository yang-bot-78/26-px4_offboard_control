// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__TRAITS_HPP_
#define RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "race_msgs/msg/detail/flight_altitude_reference__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace race_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FlightAltitudeReference & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: flight_id
  {
    out << "flight_id: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_id, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: target_agl_m
  {
    out << "target_agl_m: ";
    rosidl_generator_traits::value_to_yaml(msg.target_agl_m, out);
    out << ", ";
  }

  // member: min_agl_m
  {
    out << "min_agl_m: ";
    rosidl_generator_traits::value_to_yaml(msg.min_agl_m, out);
    out << ", ";
  }

  // member: max_agl_m
  {
    out << "max_agl_m: ";
    rosidl_generator_traits::value_to_yaml(msg.max_agl_m, out);
    out << ", ";
  }

  // member: ground_z_local_ned
  {
    out << "ground_z_local_ned: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_z_local_ned, out);
    out << ", ";
  }

  // member: target_z_local_ned
  {
    out << "target_z_local_ned: ";
    rosidl_generator_traits::value_to_yaml(msg.target_z_local_ned, out);
    out << ", ";
  }

  // member: ground_z_map
  {
    out << "ground_z_map: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_z_map, out);
    out << ", ";
  }

  // member: target_z_map
  {
    out << "target_z_map: ";
    rosidl_generator_traits::value_to_yaml(msg.target_z_map, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FlightAltitudeReference & msg,
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

  // member: flight_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flight_id: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_id, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: target_agl_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_agl_m: ";
    rosidl_generator_traits::value_to_yaml(msg.target_agl_m, out);
    out << "\n";
  }

  // member: min_agl_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_agl_m: ";
    rosidl_generator_traits::value_to_yaml(msg.min_agl_m, out);
    out << "\n";
  }

  // member: max_agl_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_agl_m: ";
    rosidl_generator_traits::value_to_yaml(msg.max_agl_m, out);
    out << "\n";
  }

  // member: ground_z_local_ned
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ground_z_local_ned: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_z_local_ned, out);
    out << "\n";
  }

  // member: target_z_local_ned
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_z_local_ned: ";
    rosidl_generator_traits::value_to_yaml(msg.target_z_local_ned, out);
    out << "\n";
  }

  // member: ground_z_map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ground_z_map: ";
    rosidl_generator_traits::value_to_yaml(msg.ground_z_map, out);
    out << "\n";
  }

  // member: target_z_map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_z_map: ";
    rosidl_generator_traits::value_to_yaml(msg.target_z_map, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FlightAltitudeReference & msg, bool use_flow_style = false)
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
  const race_msgs::msg::FlightAltitudeReference & msg,
  std::ostream & out, size_t indentation = 0)
{
  race_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use race_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const race_msgs::msg::FlightAltitudeReference & msg)
{
  return race_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<race_msgs::msg::FlightAltitudeReference>()
{
  return "race_msgs::msg::FlightAltitudeReference";
}

template<>
inline const char * name<race_msgs::msg::FlightAltitudeReference>()
{
  return "race_msgs/msg/FlightAltitudeReference";
}

template<>
struct has_fixed_size<race_msgs::msg::FlightAltitudeReference>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<race_msgs::msg::FlightAltitudeReference>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<race_msgs::msg::FlightAltitudeReference>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__TRAITS_HPP_

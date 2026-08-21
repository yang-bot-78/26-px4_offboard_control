// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fastlio_global_slam:srv/LoadMap.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__LOAD_MAP__TRAITS_HPP_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__LOAD_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "fastlio_global_slam/srv/detail/load_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace fastlio_global_slam
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadMap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: directory
  {
    out << "directory: ";
    rosidl_generator_traits::value_to_yaml(msg.directory, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: directory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "directory: ";
    rosidl_generator_traits::value_to_yaml(msg.directory, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadMap_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace fastlio_global_slam

namespace rosidl_generator_traits
{

[[deprecated("use fastlio_global_slam::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const fastlio_global_slam::srv::LoadMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  fastlio_global_slam::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fastlio_global_slam::srv::to_yaml() instead")]]
inline std::string to_yaml(const fastlio_global_slam::srv::LoadMap_Request & msg)
{
  return fastlio_global_slam::srv::to_yaml(msg);
}

template<>
inline const char * data_type<fastlio_global_slam::srv::LoadMap_Request>()
{
  return "fastlio_global_slam::srv::LoadMap_Request";
}

template<>
inline const char * name<fastlio_global_slam::srv::LoadMap_Request>()
{
  return "fastlio_global_slam/srv/LoadMap_Request";
}

template<>
struct has_fixed_size<fastlio_global_slam::srv::LoadMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<fastlio_global_slam::srv::LoadMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<fastlio_global_slam::srv::LoadMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace fastlio_global_slam
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadMap_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace fastlio_global_slam

namespace rosidl_generator_traits
{

[[deprecated("use fastlio_global_slam::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const fastlio_global_slam::srv::LoadMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  fastlio_global_slam::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fastlio_global_slam::srv::to_yaml() instead")]]
inline std::string to_yaml(const fastlio_global_slam::srv::LoadMap_Response & msg)
{
  return fastlio_global_slam::srv::to_yaml(msg);
}

template<>
inline const char * data_type<fastlio_global_slam::srv::LoadMap_Response>()
{
  return "fastlio_global_slam::srv::LoadMap_Response";
}

template<>
inline const char * name<fastlio_global_slam::srv::LoadMap_Response>()
{
  return "fastlio_global_slam/srv/LoadMap_Response";
}

template<>
struct has_fixed_size<fastlio_global_slam::srv::LoadMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<fastlio_global_slam::srv::LoadMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<fastlio_global_slam::srv::LoadMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<fastlio_global_slam::srv::LoadMap>()
{
  return "fastlio_global_slam::srv::LoadMap";
}

template<>
inline const char * name<fastlio_global_slam::srv::LoadMap>()
{
  return "fastlio_global_slam/srv/LoadMap";
}

template<>
struct has_fixed_size<fastlio_global_slam::srv::LoadMap>
  : std::integral_constant<
    bool,
    has_fixed_size<fastlio_global_slam::srv::LoadMap_Request>::value &&
    has_fixed_size<fastlio_global_slam::srv::LoadMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<fastlio_global_slam::srv::LoadMap>
  : std::integral_constant<
    bool,
    has_bounded_size<fastlio_global_slam::srv::LoadMap_Request>::value &&
    has_bounded_size<fastlio_global_slam::srv::LoadMap_Response>::value
  >
{
};

template<>
struct is_service<fastlio_global_slam::srv::LoadMap>
  : std::true_type
{
};

template<>
struct is_service_request<fastlio_global_slam::srv::LoadMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<fastlio_global_slam::srv::LoadMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__LOAD_MAP__TRAITS_HPP_

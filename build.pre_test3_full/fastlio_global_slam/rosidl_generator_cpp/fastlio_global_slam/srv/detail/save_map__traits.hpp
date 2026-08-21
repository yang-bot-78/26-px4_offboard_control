// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from fastlio_global_slam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "fastlio_global_slam/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace fastlio_global_slam
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveMap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: directory
  {
    out << "directory: ";
    rosidl_generator_traits::value_to_yaml(msg.directory, out);
    out << ", ";
  }

  // member: resolution
  {
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SaveMap_Request & msg,
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

  // member: resolution
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resolution: ";
    rosidl_generator_traits::value_to_yaml(msg.resolution, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SaveMap_Request & msg, bool use_flow_style = false)
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
  const fastlio_global_slam::srv::SaveMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  fastlio_global_slam::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fastlio_global_slam::srv::to_yaml() instead")]]
inline std::string to_yaml(const fastlio_global_slam::srv::SaveMap_Request & msg)
{
  return fastlio_global_slam::srv::to_yaml(msg);
}

template<>
inline const char * data_type<fastlio_global_slam::srv::SaveMap_Request>()
{
  return "fastlio_global_slam::srv::SaveMap_Request";
}

template<>
inline const char * name<fastlio_global_slam::srv::SaveMap_Request>()
{
  return "fastlio_global_slam/srv/SaveMap_Request";
}

template<>
struct has_fixed_size<fastlio_global_slam::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<fastlio_global_slam::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<fastlio_global_slam::srv::SaveMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace fastlio_global_slam
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveMap_Response & msg,
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
  const SaveMap_Response & msg,
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

inline std::string to_yaml(const SaveMap_Response & msg, bool use_flow_style = false)
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
  const fastlio_global_slam::srv::SaveMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  fastlio_global_slam::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use fastlio_global_slam::srv::to_yaml() instead")]]
inline std::string to_yaml(const fastlio_global_slam::srv::SaveMap_Response & msg)
{
  return fastlio_global_slam::srv::to_yaml(msg);
}

template<>
inline const char * data_type<fastlio_global_slam::srv::SaveMap_Response>()
{
  return "fastlio_global_slam::srv::SaveMap_Response";
}

template<>
inline const char * name<fastlio_global_slam::srv::SaveMap_Response>()
{
  return "fastlio_global_slam/srv/SaveMap_Response";
}

template<>
struct has_fixed_size<fastlio_global_slam::srv::SaveMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<fastlio_global_slam::srv::SaveMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<fastlio_global_slam::srv::SaveMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<fastlio_global_slam::srv::SaveMap>()
{
  return "fastlio_global_slam::srv::SaveMap";
}

template<>
inline const char * name<fastlio_global_slam::srv::SaveMap>()
{
  return "fastlio_global_slam/srv/SaveMap";
}

template<>
struct has_fixed_size<fastlio_global_slam::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_fixed_size<fastlio_global_slam::srv::SaveMap_Request>::value &&
    has_fixed_size<fastlio_global_slam::srv::SaveMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<fastlio_global_slam::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_bounded_size<fastlio_global_slam::srv::SaveMap_Request>::value &&
    has_bounded_size<fastlio_global_slam::srv::SaveMap_Response>::value
  >
{
};

template<>
struct is_service<fastlio_global_slam::srv::SaveMap>
  : std::true_type
{
};

template<>
struct is_service_request<fastlio_global_slam::srv::SaveMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<fastlio_global_slam::srv::SaveMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_

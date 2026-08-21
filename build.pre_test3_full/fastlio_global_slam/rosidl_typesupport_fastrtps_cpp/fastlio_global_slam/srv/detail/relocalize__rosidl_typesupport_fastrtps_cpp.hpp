// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from fastlio_global_slam:srv/Relocalize.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "fastlio_global_slam/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "fastlio_global_slam/srv/detail/relocalize__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace fastlio_global_slam
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
cdr_serialize(
  const fastlio_global_slam::srv::Relocalize_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  fastlio_global_slam::srv::Relocalize_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
get_serialized_size(
  const fastlio_global_slam::srv::Relocalize_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
max_serialized_size_Relocalize_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace fastlio_global_slam

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, fastlio_global_slam, srv, Relocalize_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "fastlio_global_slam/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "fastlio_global_slam/srv/detail/relocalize__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace fastlio_global_slam
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
cdr_serialize(
  const fastlio_global_slam::srv::Relocalize_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  fastlio_global_slam::srv::Relocalize_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
get_serialized_size(
  const fastlio_global_slam::srv::Relocalize_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
max_serialized_size_Relocalize_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace fastlio_global_slam

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, fastlio_global_slam, srv, Relocalize_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "fastlio_global_slam/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_fastlio_global_slam
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, fastlio_global_slam, srv, Relocalize)();

#ifdef __cplusplus
}
#endif

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
